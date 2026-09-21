#include "ipc/Recording.h"

#include "core/Log.h"

#include <cstring>

namespace fsim::ipc {

namespace {
constexpr char kRecMagic[5] = {'F', 'S', 'R', 'E', 'C'};
constexpr std::uint8_t kVersion = 1;

struct SnapshotHead {
    double simTime;
    std::uint32_t count;
};
} // namespace

RecordWriter::~RecordWriter() { close(); }

bool RecordWriter::open(const std::filesystem::path& path, const RecordingHeader& header) {
    close();
#ifdef _WIN32
    file_ = _wfopen(path.wstring().c_str(), L"wb");
#else
    file_ = std::fopen(path.string().c_str(), "wb");
#endif
    if (!file_) {
        LOG_ERROR("ipc") << "cannot write recording " << path.string();
        return false;
    }
    std::fwrite(kRecMagic, 1, 5, file_);
    std::fwrite(&kVersion, 1, 1, file_);
    std::fwrite(&header, sizeof(header), 1, file_);
    bytes_ = 6 + sizeof(header);
    LOG_INFO("ipc") << "recording to " << path.string();
    return true;
}

void RecordWriter::writeVehicle(std::uint32_t slot, const RecordedVehicle& row) {
    if (!file_) return;
    const std::uint8_t kind = 'T';
    std::fwrite(&kind, 1, 1, file_);
    std::fwrite(&slot, sizeof(slot), 1, file_);
    std::fwrite(&row, sizeof(row), 1, file_);
    bytes_ += 1 + sizeof(slot) + sizeof(row);
}

void RecordWriter::writeSnapshot(double simTime, const std::vector<std::pair<std::uint32_t, VehicleSample>>& samples) {
    if (!file_) return;
    const std::uint8_t kind = 'S';
    const SnapshotHead head{simTime, static_cast<std::uint32_t>(samples.size())};
    std::fwrite(&kind, 1, 1, file_);
    std::fwrite(&head, sizeof(head), 1, file_);
    for (const auto& [slot, sample] : samples) {
        std::fwrite(&slot, sizeof(slot), 1, file_);
        std::fwrite(&sample, sizeof(sample), 1, file_);
    }
    bytes_ += 1 + sizeof(head) + samples.size() * (sizeof(std::uint32_t) + sizeof(VehicleSample));
}

void RecordWriter::close() {
    if (file_) std::fclose(file_);
    file_ = nullptr;
}

bool Recording::load(const std::filesystem::path& path, std::string* error) {
    frames_.clear();
#ifdef _WIN32
    std::FILE* f = _wfopen(path.wstring().c_str(), L"rb");
#else
    std::FILE* f = std::fopen(path.string().c_str(), "rb");
#endif
    if (!f) {
        if (error) *error = "cannot open " + path.string();
        return false;
    }
    char magic[5];
    std::uint8_t version = 0;
    if (std::fread(magic, 1, 5, f) != 5 || std::memcmp(magic, kRecMagic, 5) != 0 || std::fread(&version, 1, 1, f) != 1 || version != kVersion ||
        std::fread(&header_, sizeof(header_), 1, f) != 1) {
        std::fclose(f);
        if (error) *error = "not an fsim recording: " + path.string();
        return false;
    }
    Frame pending; // table changes accumulate until the next snapshot
    for (;;) {
        std::uint8_t kind = 0;
        if (std::fread(&kind, 1, 1, f) != 1) break;
        if (kind == 'T') {
            std::uint32_t slot = 0;
            RecordedVehicle row;
            if (std::fread(&slot, sizeof(slot), 1, f) != 1 || std::fread(&row, sizeof(row), 1, f) != 1) break;
            pending.tableChanges.emplace_back(slot, row);
        } else if (kind == 'S') {
            SnapshotHead head{};
            if (std::fread(&head, sizeof(head), 1, f) != 1) break;
            Frame frame;
            frame.simTime = head.simTime;
            frame.tableChanges.swap(pending.tableChanges);
            frame.samples.resize(head.count);
            bool ok = true;
            for (auto& [slot, sample] : frame.samples)
                if (std::fread(&slot, sizeof(slot), 1, f) != 1 || std::fread(&sample, sizeof(sample), 1, f) != 1) { ok = false; break; }
            if (!ok) break;
            frames_.push_back(std::move(frame));
        } else {
            break; // unknown record: stop at what we have
        }
    }
    std::fclose(f);
    if (!pending.tableChanges.empty()) {
        if (frames_.empty()) frames_.push_back(Frame{});
        auto& last = frames_.back().tableChanges;
        last.insert(last.end(), pending.tableChanges.begin(), pending.tableChanges.end());
    }
    LOG_INFO("ipc") << "recording " << path.string() << ": " << frames_.size() << " frames, " << duration() << " s";
    return !frames_.empty();
}

} // namespace fsim::ipc
