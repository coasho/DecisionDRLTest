#include "ipc/WorldPublisher.h"

#include "core/Log.h"
#include "ipc/WorldRegistry.h"
#include "platform/Clock.h"

#include <algorithm>
#include <cstring>
#include <new>

namespace fsim::ipc {

namespace {

void copyString(char* dst, std::size_t capacity, const char* src) noexcept {
    std::memset(dst, 0, capacity);
    if (src) std::memcpy(dst, src, std::min(std::strlen(src), capacity - 1));
}

} // namespace

WorldPublisher::WorldPublisher(const Options& options) : options_(options) {
    options_.capacity = std::max<std::uint32_t>(1, options_.capacity);
    const std::size_t size = Layout::totalSize(options_.capacity);
    const std::string mapped = std::string(kWorldPrefix) + options_.name;
    if (!memory_.create(mapped, size)) {
        LOG_WARN("ipc") << "could not create shared memory '" << mapped << "' (" << size << " bytes); world '" << options_.name
                        << "' will not be visible to viewers";
        return;
    }
    base_ = static_cast<unsigned char*>(memory_.data());
    auto* h = reinterpret_cast<WorldHeader*>(base_);
    if (!memory_.created() && h->magic == kMagic && h->alive.load(std::memory_order_acquire) != 0 &&
        platform::processAlive(h->publisherPid)) {
        LOG_WARN("ipc") << "world '" << options_.name << "' is already published by process " << h->publisherPid
                        << "; this world will not be visible to viewers";
        base_ = nullptr;
        memory_.close();
        return;
    }
    // Fresh or abandoned mapping: (re)initialise in place. Readers tolerate
    // this because every field they use is guarded by a sequence or the magic.
    h->magic = 0;
    std::atomic_thread_fence(std::memory_order_seq_cst);
    std::memset(base_, 0, size);
    new (h) WorldHeader();
    h->layoutVersion = kLayoutVersion;
    h->headerSize = sizeof(WorldHeader);
    h->capacity = options_.capacity;
    h->publisherPid = platform::currentProcessId();
    copyString(h->name, kNameLength, options_.name.c_str());
    h->dt = options_.dt;
    h->frameSkip = options_.frameSkip;
    h->tableOffset = Layout::tableOffset();
    h->slotOffset = Layout::slotOffset(options_.capacity);
    h->slotStride = Layout::slotStride(options_.capacity);
    h->totalSize = size;
    for (std::uint32_t i = 0; i < options_.capacity; ++i) new (base_ + h->tableOffset + sizeof(VehicleRecord) * i) VehicleRecord();
    for (std::uint32_t s = 0; s < kSlots; ++s) new (base_ + h->slotOffset + h->slotStride * s) StateSlotHeader();
    h->alive.store(1, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_seq_cst);
    h->magic = kMagic;
    header_ = h;
    writeSlot_ = 1;
    WorldRegistry::add(options_.name);
    LOG_INFO("ipc") << "publishing world '" << options_.name << "' (" << options_.capacity << " vehicle slots, " << size / 1024
                    << " KiB) for viewers";
}

WorldPublisher::~WorldPublisher() {
    if (!header_) return;
    header_->alive.store(0, std::memory_order_release);
    WorldRegistry::remove(options_.name);
}

VehicleRecord* WorldPublisher::row(std::uint32_t index) noexcept {
    return reinterpret_cast<VehicleRecord*>(base_ + header_->tableOffset + sizeof(VehicleRecord) * index);
}

StateSlotHeader* WorldPublisher::slot(std::uint32_t index) noexcept {
    return reinterpret_cast<StateSlotHeader*>(base_ + header_->slotOffset + header_->slotStride * index);
}

VehicleSample* WorldPublisher::samples(std::uint32_t index) noexcept {
    return reinterpret_cast<VehicleSample*>(base_ + header_->slotOffset + header_->slotStride * index + Layout::align(sizeof(StateSlotHeader)));
}

void WorldPublisher::setVehicle(std::uint32_t index, const VehicleRecord& record) noexcept {
    if (!header_ || index >= header_->capacity) return;
    VehicleRecord* r = row(index);
    r->seq.fetch_add(1, std::memory_order_acq_rel);
    r->id = record.id;
    r->generation = record.generation;
    r->alive = record.alive;
    r->controlLevel = record.controlLevel;
    std::memcpy(r->name, record.name, kNameLength);
    std::memcpy(r->type, record.type, kTypeLength);
    std::memcpy(r->model, record.model, kPathLength);
    r->initialLatitudeDeg = record.initialLatitudeDeg;
    r->initialLongitudeDeg = record.initialLongitudeDeg;
    r->initialAltitudeMslM = record.initialAltitudeMslM;
    r->initialHeadingDeg = record.initialHeadingDeg;
    r->seq.fetch_add(1, std::memory_order_acq_rel);
    header_->tableGeneration.fetch_add(1, std::memory_order_acq_rel);
}

void WorldPublisher::setControlLevel(std::uint32_t index, std::uint8_t level) noexcept {
    if (!header_ || index >= header_->capacity) return;
    VehicleRecord* r = row(index);
    if (r->controlLevel == level) return;
    r->seq.fetch_add(1, std::memory_order_acq_rel);
    r->controlLevel = level;
    r->seq.fetch_add(1, std::memory_order_acq_rel);
    header_->tableGeneration.fetch_add(1, std::memory_order_acq_rel);
}

void WorldPublisher::clearVehicle(std::uint32_t index) noexcept {
    if (!header_ || index >= header_->capacity) return;
    VehicleRecord* r = row(index);
    r->seq.fetch_add(1, std::memory_order_acq_rel);
    r->alive = 0;
    ++r->generation;
    r->seq.fetch_add(1, std::memory_order_acq_rel);
    header_->tableGeneration.fetch_add(1, std::memory_order_acq_rel);
}

void WorldPublisher::setEnvironment(const sim::EnvironmentState& environment) noexcept {
    if (!header_) return;
    header_->environmentSeq.fetch_add(1, std::memory_order_acq_rel);
    header_->environment = environment;
    header_->environmentSeq.fetch_add(1, std::memory_order_acq_rel);
}

bool WorldPublisher::publish(double simTime, Span<const sim::VehicleState> states, Span<const sim::ControlInputs> inputs,
                             bool force) noexcept {
    if (!header_) return false;
    const std::int64_t now = platform::Clock::nanoseconds();
    if (!force && static_cast<double>(now - lastPublishNs_) * 1e-9 < options_.publishIntervalSeconds) return false;
    lastPublishNs_ = now;

    // Rotate through the slots, skipping the latest one (readers copy from it;
    // a reader still copying an older slot simply retries its seqlock).
    const std::uint32_t latest = header_->latestSlot.load(std::memory_order_acquire);
    std::uint32_t target = (writeSlot_ + 1) % kSlots;
    if (target == latest) target = (target + 1) % kSlots;

    StateSlotHeader* s = slot(target);
    VehicleSample* v = samples(target);
    const std::uint32_t n = static_cast<std::uint32_t>(std::min<std::size_t>(states.size(), header_->capacity));
    s->seq.fetch_add(1, std::memory_order_acq_rel);
    s->sequence = header_->publishSequence.load(std::memory_order_relaxed) + 1;
    s->simTime = simTime;
    s->wallNs = now;
    for (std::uint32_t i = 0; i < n; ++i) {
        v[i].state = states[i];
        if (i < inputs.size()) v[i].inputs = inputs[i];
    }
    s->seq.fetch_add(1, std::memory_order_acq_rel);
    header_->publishSequence.store(s->sequence, std::memory_order_release);
    header_->lastPublishWallNs.store(now, std::memory_order_release);
    header_->latestSlot.store(target, std::memory_order_release);
    writeSlot_ = target;
    return true;
}

void WorldPublisher::addSteps(std::uint64_t vehicleSteps, std::uint64_t worldSteps) noexcept {
    if (!header_) return;
    header_->vehicleSteps.fetch_add(vehicleSteps, std::memory_order_relaxed);
    header_->worldSteps.fetch_add(worldSteps, std::memory_order_relaxed);
}

} // namespace fsim::ipc
