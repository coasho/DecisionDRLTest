#pragma once

// Recording of a world to a file and playback (design 9.5 "Recorder"): the
// same vehicle table rows and samples the shared-memory segment carries,
// appended at a fixed simulation-time interval. Format v1 (little-endian):
//   "FSREC" u8 version=1 | RecordingHeader | records...
//   record: u8 kind ('T' table row, 'S' snapshot) followed by
//     'T': u32 slot, VehicleRecord (without the seqlock word)
//     'S': SnapshotHead{double simTime; u32 count}, count x {u32 slot, VehicleSample}

#include "ipc/WorldLayout.h"

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace fsim::ipc {

struct RecordingHeader {
    std::uint32_t capacity = 0;
    double dt = 0.0;
    std::int32_t frameSkip = 1;
    char name[kNameLength] = {};
};

/// Plain-data copy of a table row (no atomics) as stored in the file.
struct RecordedVehicle {
    std::uint32_t id = 0;
    std::uint64_t generation = 0;
    std::uint8_t alive = 0;
    std::uint8_t controlLevel = 0;
    char name[kNameLength] = {};
    char type[kTypeLength] = {};
    char model[kPathLength] = {};
    double initialLatitudeDeg = 0.0, initialLongitudeDeg = 0.0, initialAltitudeMslM = 0.0, initialHeadingDeg = 0.0;
};

class RecordWriter {
public:
    RecordWriter() = default;
    ~RecordWriter();
    RecordWriter(const RecordWriter&) = delete;
    RecordWriter& operator=(const RecordWriter&) = delete;

    bool open(const std::filesystem::path& path, const RecordingHeader& header);
    bool isOpen() const noexcept { return file_ != nullptr; }
    void writeVehicle(std::uint32_t slot, const RecordedVehicle& row);
    /// `samples` are the alive slots only (slot index + sample).
    void writeSnapshot(double simTime, const std::vector<std::pair<std::uint32_t, VehicleSample>>& samples);
    void close();
    std::uint64_t bytesWritten() const noexcept { return bytes_; }

private:
    std::FILE* file_ = nullptr;
    std::uint64_t bytes_ = 0;
};

/// Whole recording in memory, frames indexed by simulation time.
class Recording {
public:
    struct Frame {
        double simTime = 0.0;
        std::vector<std::pair<std::uint32_t, VehicleSample>> samples;
        /// Table changes that happened at or before this frame since the previous one.
        std::vector<std::pair<std::uint32_t, RecordedVehicle>> tableChanges;
    };

    bool load(const std::filesystem::path& path, std::string* error = nullptr);
    const RecordingHeader& header() const noexcept { return header_; }
    const std::vector<Frame>& frames() const noexcept { return frames_; }
    double duration() const noexcept { return frames_.empty() ? 0.0 : frames_.back().simTime - frames_.front().simTime; }

private:
    RecordingHeader header_;
    std::vector<Frame> frames_;
};

} // namespace fsim::ipc
