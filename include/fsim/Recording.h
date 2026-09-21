#pragma once

// fsim SDK: reading recordings (design 9.5 "Recorder"). A world with
// `WorldOptions::recordPath` writes every vehicle's state and actuator inputs
// per step to an .fsrec file; besides replaying it in the viewer, a trainer
// can read it back as data - trajectories of scripted behaviours for
// imitation learning, evaluation logs, or regression baselines.
//
//   fsim::Recording rec = fsim::Recording::load("run.fsrec");
//   for (const auto& frame : rec.frames())
//       for (const auto& s : frame.samples)   // one per live vehicle, by slot
//           learner.add(s.state, s.inputs);   // truth state, actuator inputs

#include "fsim/ControlInputs.h"
#include "fsim/Export.h"
#include "fsim/VehicleState.h"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace fsim {

class FSIM_API Recording {
public:
    struct Sample {
        std::uint32_t slot = 0;   ///< the vehicle's slot in the world (stable while it lives)
        VehicleState state;       ///< truth
        ControlInputs inputs;     ///< actuator inputs the cascade produced for this step
    };
    /// A vehicle's table row as it was written: creation/reset (alive) or removal.
    struct VehicleEvent {
        std::uint32_t slot = 0;
        std::uint32_t id = 0;
        std::uint64_t generation = 0;
        bool alive = false;
        std::uint8_t controlLevel = 0; ///< control::Level
        std::string name, type, model;
        double initialLatitudeDeg = 0.0, initialLongitudeDeg = 0.0, initialAltitudeMslM = 0.0, initialHeadingDeg = 0.0;
    };
    struct Frame {
        double simTime = 0.0;
        std::vector<Sample> samples;
        std::vector<VehicleEvent> events; ///< table changes since the previous frame (creations, resets, level changes, removals)
    };

    /// Throws fsim::Error when the file cannot be read.
    static Recording load(const std::filesystem::path& path);

    const std::vector<Frame>& frames() const noexcept { return frames_; }
    double duration() const noexcept { return frames_.empty() ? 0.0 : frames_.back().simTime - frames_.front().simTime; }
    double dt() const noexcept { return dt_; }
    int frameSkip() const noexcept { return frameSkip_; }
    const std::string& worldName() const noexcept { return worldName_; }
    std::uint32_t capacity() const noexcept { return capacity_; }

    /// The samples of one vehicle (by slot) across every frame, with the frame's time.
    struct Track {
        std::vector<double> simTime;
        std::vector<VehicleState> states;
        std::vector<ControlInputs> inputs;
    };
    Track track(std::uint32_t slot) const;

private:
    std::vector<Frame> frames_;
    double dt_ = 0.0;
    int frameSkip_ = 1;
    std::string worldName_;
    std::uint32_t capacity_ = 0;
};

} // namespace fsim
