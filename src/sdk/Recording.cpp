// fsim SDK: recordings read back as data (over ipc::Recording).

#include "fsim/Recording.h"

#include "fsim/World.h"
#include "ipc/Recording.h"

#include <cstring>

namespace fsim {

Recording Recording::load(const std::filesystem::path& path) {
    ipc::Recording raw;
    std::string error;
    if (!raw.load(path, &error)) throw Error("Recording::load: " + error);
    Recording rec;
    rec.dt_ = raw.header().dt;
    rec.frameSkip_ = raw.header().frameSkip;
    rec.capacity_ = raw.header().capacity;
    rec.worldName_.assign(raw.header().name, ::strnlen(raw.header().name, ipc::kNameLength));
    rec.frames_.reserve(raw.frames().size());
    for (const auto& f : raw.frames()) {
        Frame frame;
        frame.simTime = f.simTime;
        frame.samples.reserve(f.samples.size());
        for (const auto& [slot, sample] : f.samples) {
            Sample s;
            s.slot = slot;
            s.state = sample.state;
            s.inputs = sample.inputs;
            frame.samples.push_back(s);
        }
        for (const auto& [slot, row] : f.tableChanges) {
            VehicleEvent e;
            e.slot = slot;
            e.id = row.id;
            e.generation = row.generation;
            e.alive = row.alive != 0;
            e.controlLevel = row.controlLevel;
            e.name.assign(row.name, ::strnlen(row.name, ipc::kNameLength));
            e.type.assign(row.type, ::strnlen(row.type, ipc::kTypeLength));
            e.model.assign(row.model, ::strnlen(row.model, ipc::kPathLength));
            e.initialLatitudeDeg = row.initialLatitudeDeg;
            e.initialLongitudeDeg = row.initialLongitudeDeg;
            e.initialAltitudeMslM = row.initialAltitudeMslM;
            e.initialHeadingDeg = row.initialHeadingDeg;
            frame.events.push_back(std::move(e));
        }
        rec.frames_.push_back(std::move(frame));
    }
    return rec;
}

Recording::Track Recording::track(std::uint32_t slot) const {
    Track t;
    for (const auto& f : frames_)
        for (const auto& s : f.samples)
            if (s.slot == slot) {
                t.simTime.push_back(f.simTime);
                t.states.push_back(s.state);
                t.inputs.push_back(s.inputs);
                break;
            }
    return t;
}

} // namespace fsim
