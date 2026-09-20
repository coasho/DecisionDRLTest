#include "ipc/WorldRegistry.h"

#include "ipc/WorldLayout.h"
#include "platform/Clock.h"
#include "platform/SharedMemory.h"

#include <algorithm>
#include <cstring>
#include <thread>

namespace fsim::ipc {

namespace {

// The registry mapping must outlive the publisher that created it while any
// other process may still look at it, so keep one handle per process.
platform::SharedMemory& mapping() {
    static platform::SharedMemory memory;
    return memory;
}

RegistryHeader* openRegistry(bool create) noexcept {
    auto& m = mapping();
    if (!m.valid()) {
        const bool ok = create ? m.create(kRegistryName, sizeof(RegistryHeader)) : m.open(kRegistryName, sizeof(RegistryHeader));
        if (!ok) return nullptr;
    }
    auto* h = static_cast<RegistryHeader*>(m.data());
    if (m.created() || h->magic == 0) {
        // Fresh (zero-filled) mapping: stamp it. A racing creator stamps the same values.
        h->layoutVersion = kLayoutVersion;
        h->magic = kMagic;
    }
    return h->magic == kMagic ? h : nullptr;
}

struct SpinLock {
    std::atomic<std::uint32_t>& lock;
    explicit SpinLock(std::atomic<std::uint32_t>& l) noexcept : lock(l) {
        for (int spins = 0; lock.exchange(1, std::memory_order_acquire) != 0; ++spins) {
            if (spins > 1000) std::this_thread::yield();
            if (spins > 100000) break; // a dead holder: take it anyway
        }
    }
    ~SpinLock() { lock.store(0, std::memory_order_release); }
};

void copyName(char* dst, const std::string& src) noexcept {
    std::memset(dst, 0, kNameLength);
    std::memcpy(dst, src.data(), std::min<std::size_t>(src.size(), kNameLength - 1));
}

void prune(RegistryHeader& h) noexcept {
    for (auto& e : h.entries)
        if (e.pid != 0 && !platform::processAlive(e.pid)) {
            e.seq.fetch_add(1, std::memory_order_acq_rel);
            e.pid = 0;
            e.name[0] = '\0';
            e.seq.fetch_add(1, std::memory_order_acq_rel);
        }
}

} // namespace

bool WorldRegistry::add(const std::string& name) noexcept {
    auto* h = openRegistry(true);
    if (!h) return false;
    SpinLock guard(h->lock);
    prune(*h);
    RegistryEntry* free = nullptr;
    for (auto& e : h->entries) {
        if (e.pid != 0 && std::strncmp(e.name, name.c_str(), kNameLength) == 0) { free = &e; break; } // re-register
        if (!free && e.pid == 0) free = &e;
    }
    if (!free) return false;
    free->seq.fetch_add(1, std::memory_order_acq_rel);
    free->pid = platform::currentProcessId();
    free->createdWallNs = platform::Clock::nanoseconds();
    copyName(free->name, name);
    free->seq.fetch_add(1, std::memory_order_acq_rel);
    return true;
}

void WorldRegistry::remove(const std::string& name) noexcept {
    auto* h = openRegistry(false);
    if (!h) return;
    SpinLock guard(h->lock);
    const auto pid = platform::currentProcessId();
    for (auto& e : h->entries)
        if (e.pid == pid && std::strncmp(e.name, name.c_str(), kNameLength) == 0) {
            e.seq.fetch_add(1, std::memory_order_acq_rel);
            e.pid = 0;
            e.name[0] = '\0';
            e.seq.fetch_add(1, std::memory_order_acq_rel);
        }
}

std::vector<WorldInfo> WorldRegistry::list() noexcept {
    std::vector<WorldInfo> out;
    auto* h = openRegistry(false);
    if (!h) return out;
    for (auto& e : h->entries) {
        RegistryEntry copy;
        for (int attempt = 0; attempt < 8; ++attempt) {
            const auto s0 = e.seq.load(std::memory_order_acquire);
            if (s0 & 1u) continue;
            copy.pid = e.pid;
            copy.createdWallNs = e.createdWallNs;
            std::memcpy(copy.name, e.name, kNameLength);
            std::atomic_thread_fence(std::memory_order_acquire);
            if (e.seq.load(std::memory_order_acquire) == s0) break;
        }
        if (copy.pid == 0 || copy.name[0] == '\0') continue;
        WorldInfo info;
        info.name.assign(copy.name, ::strnlen(copy.name, kNameLength));
        info.pid = copy.pid;
        info.createdWallNs = copy.createdWallNs;
        info.alive = platform::processAlive(copy.pid);
        if (info.alive) out.push_back(std::move(info));
    }
    std::sort(out.begin(), out.end(), [](const WorldInfo& a, const WorldInfo& b) { return a.createdWallNs > b.createdWallNs; });
    return out;
}

} // namespace fsim::ipc
