// fsim_control_bench: the control architecture's measurements
// (docs/control-architecture.md, section 12).
//
//   fsim_control_bench micro          ControlStack::update, ns per update, per level and behaviour,
//                                     with the axes owned apart, the vehicle default's hold, and
//                                     envelope protection limiting (", limit") or reporting (", report")
//   fsim_control_bench command        World::command (the legacy path), ns per call
//   fsim_control_bench world [off]    vehicle-steps/s with every vehicle commanded every step; off: every
//                                     vehicle's envelope protection off (the designs have it on by default),
//                                     for the protection gate: protected throughput against unprotected
//   fsim_control_bench alloc          heap allocations in the steady state (exit 1 if any)
//   fsim_control_bench digest FILE [off]   per-flight digests of every world step (compare across builds);
//                                     off: every vehicle's envelope protection off
//   fsim_control_bench checkpoints FILE   the stock c172x checkpoints (tests/data)
//
// Heap allocations are counted by redirecting the malloc, calloc and realloc
// imports of this executable and of libstdc++ (whose operator new calls
// malloc), so an allocation made anywhere in the platform's code - inlined
// templates or the library's own string members - is seen.

#include "control_flights.h"

#include "control/Runtime.h"
#include "fsim/BuiltinControllers.h"
#include "fsim/ControlStack.h"
#include "core/Log.h"
#include "platform/Threads.h"
#include "session/World.h"

#include <windows.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::control;

namespace {

// --- allocation counting ---------------------------------------------------------

std::atomic<std::uint64_t> gAllocations{0};  ///< the platform's own
std::atomic<std::uint64_t> gFlightModel{0};  ///< made inside JSBSim (it allocates as it steps)
std::atomic<bool> gCounting{false};
std::uintptr_t gJsbsimBegin = 0, gJsbsimEnd = 0;

/// Attribute an allocation: JSBSim's if any caller up the stack is in its DLL.
void countAllocation() {
    void* frames[24];
    const USHORT n = RtlCaptureStackBackTrace(2, 24, frames, nullptr);
    for (USHORT i = 0; i < n; ++i) {
        const auto a = reinterpret_cast<std::uintptr_t>(frames[i]);
        if (a >= gJsbsimBegin && a < gJsbsimEnd) {
            gFlightModel.fetch_add(1, std::memory_order_relaxed);
            return;
        }
    }
    gAllocations.fetch_add(1, std::memory_order_relaxed);
}

using MallocFn = void*(__cdecl*)(size_t);
using CallocFn = void*(__cdecl*)(size_t, size_t);
using ReallocFn = void*(__cdecl*)(void*, size_t);
MallocFn realMalloc = nullptr;
CallocFn realCalloc = nullptr;
ReallocFn realRealloc = nullptr;

void* __cdecl countingMalloc(size_t n) {
    if (gCounting.load(std::memory_order_relaxed)) countAllocation();
    return realMalloc(n);
}
void* __cdecl countingCalloc(size_t n, size_t size) {
    if (gCounting.load(std::memory_order_relaxed)) countAllocation();
    return realCalloc(n, size);
}
void* __cdecl countingRealloc(void* p, size_t n) {
    if (gCounting.load(std::memory_order_relaxed)) countAllocation();
    return realRealloc(p, n);
}

/// Point a module's malloc/calloc/realloc imports at the counters. Returns how many it redirected.
int redirectImports(HMODULE module) {
    if (!module) return 0;
    auto* base = reinterpret_cast<BYTE*>(module);
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
    const auto& dir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
    if (!dir.VirtualAddress) return 0;
    int redirected = 0;
    for (auto* imp = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR*>(base + dir.VirtualAddress); imp->Name; ++imp) {
        auto* thunk = reinterpret_cast<IMAGE_THUNK_DATA*>(base + imp->FirstThunk);
        const auto* names = reinterpret_cast<const IMAGE_THUNK_DATA*>(base + (imp->OriginalFirstThunk ? imp->OriginalFirstThunk : imp->FirstThunk));
        for (; names->u1.AddressOfData; ++names, ++thunk) {
            if (IMAGE_SNAP_BY_ORDINAL(names->u1.Ordinal)) continue;
            const char* fn = reinterpret_cast<const char*>(reinterpret_cast<const IMAGE_IMPORT_BY_NAME*>(base + names->u1.AddressOfData)->Name);
            ULONG_PTR replacement = 0;
            if (!std::strcmp(fn, "malloc")) {
                if (!realMalloc) realMalloc = reinterpret_cast<MallocFn>(thunk->u1.Function);
                replacement = reinterpret_cast<ULONG_PTR>(&countingMalloc);
            } else if (!std::strcmp(fn, "calloc")) {
                if (!realCalloc) realCalloc = reinterpret_cast<CallocFn>(thunk->u1.Function);
                replacement = reinterpret_cast<ULONG_PTR>(&countingCalloc);
            } else if (!std::strcmp(fn, "realloc")) {
                if (!realRealloc) realRealloc = reinterpret_cast<ReallocFn>(thunk->u1.Function);
                replacement = reinterpret_cast<ULONG_PTR>(&countingRealloc);
            }
            if (!replacement || thunk->u1.Function == replacement) continue;
            DWORD old = 0;
            VirtualProtect(&thunk->u1.Function, sizeof(ULONG_PTR), PAGE_READWRITE, &old);
            thunk->u1.Function = replacement;
            VirtualProtect(&thunk->u1.Function, sizeof(ULONG_PTR), old, &old);
            ++redirected;
        }
    }
    return redirected;
}

bool installAllocationCounter() {
    if (HMODULE jsbsim = GetModuleHandleW(L"libJSBSim.dll")) {
        const auto* base = reinterpret_cast<const BYTE*>(jsbsim);
        const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS*>(base + reinterpret_cast<const IMAGE_DOS_HEADER*>(base)->e_lfanew);
        gJsbsimBegin = reinterpret_cast<std::uintptr_t>(base);
        gJsbsimEnd = gJsbsimBegin + nt->OptionalHeader.SizeOfImage;
    }
    const int exe = redirectImports(GetModuleHandleW(nullptr));
    const int stdcxx = redirectImports(GetModuleHandleW(L"libstdc++-6.dll"));
    if (!realMalloc || stdcxx == 0) {
        std::fprintf(stderr, "allocation counter: could not redirect malloc (exe %d, libstdc++ %d imports)\n", exe, stdcxx);
        return false;
    }
    return true;
}

struct Counting {
    std::uint64_t start, startFlightModel;
    Counting() : start(gAllocations.load()), startFlightModel(gFlightModel.load()) { gCounting = true; }
    ~Counting() { gCounting = false; }
    std::uint64_t count() const { return gAllocations.load() - start; }
    std::uint64_t flightModel() const { return gFlightModel.load() - startFlightModel; }
};

// --- the cases ---------------------------------------------------------------------

struct Case {
    const char* name;
    std::function<Command(const sim::VehicleState&)> command;
    /// Instead of a command: the runtime's configuration written as a host would (step 3's cases).
    std::function<void(RuntimeConfig&)> configure = nullptr;
    /// The attitude level's controller, when not the default (step 5b's cases).
    const char* attitudeLoop = nullptr;
};

/// Envelope protection with limits the synthetic flight (advance()) keeps
/// running into: bank, pitch, alpha and the load factor all engage part of the time.
void protect(RuntimeConfig& c, ProtectionMode mode) {
    Protection& p = c.protection;
    p.mode = mode;
    EnvelopeLimits& e = p.clean;
    e.bankMaxRad = 0.15, e.pitchMinRad = -0.2, e.pitchMaxRad = 0.04;
    e.alphaMaxRad = 0.055, e.loadFactorMin = -1.0, e.loadFactorMax = 1.4;
    e.rollRateMaxRadS = 0.15, e.casMinMs = 54.0, e.casMaxMs = 80.0, e.machMax = 0.5;
    p.flaps = e;
    p.alphaZeroLiftRad = -0.05;
    p.pitchSurface = true;
    p.elevatorGainG = 4.0, p.elevatorGainCasMs = 55.0;
}

/// A hangar direct design's envelope - alpha_max and its stall speed - that
/// the synthetic flight stays well inside: protection's cost in ordinary flight.
void protectDesign(RuntimeConfig& c) {
    Protection& p = c.protection;
    p.mode = ProtectionMode::Limit;
    p.clean.alphaMaxRad = 0.3, p.clean.casMinMs = 30.0;
    p.flaps = p.clean;
    p.alphaZeroLiftRad = -0.05;
    p.pitchSurface = true;
    p.elevatorGainG = 4.0, p.elevatorGainCasMs = 55.0;
}

/// A slot given to an activity, as CapabilityHost::submit writes it.
void assign(RuntimeConfig& c, std::size_t slot, const Command& command, AxisMask axes) {
    SetpointSlot& s = c.slots[slot];
    s.command = command;
    s.level = levelOf(command);
    s.axes = axes;
    ++s.generation;
    ++s.revision;
    for (std::size_t a = 0; a < kAxisCount; ++a)
        if (axes & (1u << a)) c.owner[a] = static_cast<std::uint8_t>(slot);
    ++c.revision;
}

sim::VehicleState levelFlight() {
    sim::VehicleState s;
    s.latitudeRad = flights::kLatDeg * flights::kDeg;
    s.longitudeRad = flights::kLonDeg * flights::kDeg;
    s.altitudeMslM = s.altitudeAglM = 1500.0;
    s.airspeedTrueMs = s.airspeedCalibratedMs = 55.0;
    s.velocityBodyMs[0] = 55.0;
    s.velocityNedMs[0] = 55.0;
    s.loadFactor = 1.0;
    s.engineCount = 1;
    return s;
}

std::vector<Case> cases() {
    std::vector<Case> out;
    out.push_back({"actuator", [](const sim::VehicleState&) { return Command(ActuatorCommand{0.1, -0.05, 0.0, 0.6}); }});
    out.push_back({"attitude", [](const sim::VehicleState&) { return Command(AttitudeCommand{0.2, 0.05, kHold, 0.785, kHold, 55.0}); }});
    out.push_back({"acceleration", [](const sim::VehicleState&) { return Command(AccelerationCommand{1.5, 0.2, kHold, 0.6}); }});
    out.push_back({"velocity", [](const sim::VehicleState&) { return Command(VelocityCommand{60.0, 2.0, 0.5, kHold}); }});
    out.push_back({"position", [](const sim::VehicleState& s) {
                       return Command(PositionCommand{s.latitudeRad + 0.01, s.longitudeRad + 0.01, 1600.0, 60.0, 200.0});
                   }});
    out.push_back({"hold", [](const sim::VehicleState&) { return Command(flights::behavior("hold")); }});
    out.push_back({"loiter", [](const sim::VehicleState& s) {
                       auto b = flights::behavior("loiter");
                       b.params = {{"radius_m", 800.0}, {"altitude_m", 1600.0}, {"lat_deg", s.latitudeRad / flights::kDeg + 0.01},
                                   {"lon_deg", s.longitudeRad / flights::kDeg}};
                       return Command(b);
                   }});
    out.push_back({"waypoints", [](const sim::VehicleState& s) {
                       auto b = flights::behavior("waypoints");
                       for (int i = 1; i <= 3; ++i)
                           b.points.push_back(PositionCommand{s.latitudeRad + 0.01 * i, s.longitudeRad, 1500.0, 55.0, 150.0});
                       b.params = {{"loop", 1.0}};
                       return Command(b);
                   }});
    // step 3: a policy's bank beside an autopilot's height and speed, merged down the cascade
    out.push_back({"axes apart", nullptr, [](RuntimeConfig& c) {
                       assign(c, 0, AttitudeCommand{0.2, kHold, kHold, 0.785, kHold, kHold}, kLateral);
                       assign(c, 1, VelocityCommand{60.0, 2.0, kHold, kHold}, axisBit(Axis::Pitch) | axisBit(Axis::Thrust));
                   }});
    // nothing owned, the vehicle default a hold
    out.push_back({"default hold", nullptr, [](RuntimeConfig& c) { c.vehicleDefault = VehicleDefault::Hold; }});
    // step 4: envelope protection on the whole-vehicle commands above - a design's envelope, well inside it
    out.push_back({"attitude, design", nullptr, [](RuntimeConfig& c) {
                       protectDesign(c);
                       assign(c, 0, AttitudeCommand{0.2, 0.05, kHold, 0.785, kHold, 55.0}, kLegacyAxes);
                   }});
    out.push_back({"velocity, design", nullptr, [](RuntimeConfig& c) {
                       protectDesign(c);
                       assign(c, 0, VelocityCommand{60.0, 2.0, 0.5, kHold}, kLegacyAxes);
                   }});
    out.push_back({"actuator, design", nullptr, [](RuntimeConfig& c) {
                       protectDesign(c);
                       assign(c, 0, ActuatorCommand{0.1, -0.05, 0.0, 0.6}, kLegacyAxes);
                   }});
    // step 5b: a design's attitude flown over pseudo-controls, allocated at the
    // acceleration level - the attitude, velocity and merged cases above, as a hangar design flies them
    out.push_back({"attitude, pseudo", nullptr,
                   [](RuntimeConfig& c) {
                       protectDesign(c);
                       assign(c, 0, AttitudeCommand{0.2, 0.05, kHold, 0.785, kHold, 55.0}, kLegacyAxes);
                   },
                   "pseudo_attitude"});
    out.push_back({"velocity, pseudo", nullptr,
                   [](RuntimeConfig& c) {
                       protectDesign(c);
                       assign(c, 0, VelocityCommand{60.0, 2.0, 0.5, kHold}, kLegacyAxes);
                   },
                   "pseudo_attitude"});
    out.push_back({"apart, pseudo", nullptr,
                   [](RuntimeConfig& c) {
                       assign(c, 0, AccelerationCommand{kHold, 0.2, kHold, kHold}, kLateral);
                       assign(c, 1, VelocityCommand{60.0, 2.0, kHold, kHold}, axisBit(Axis::Pitch) | axisBit(Axis::Thrust));
                   },
                   "pseudo_attitude"});
    // ...and every limit given, the flight running into them all the time: the worst case
    for (const ProtectionMode mode : {ProtectionMode::Limit, ProtectionMode::Report}) {
        const bool limit = mode == ProtectionMode::Limit;
        out.push_back({limit ? "attitude, limit" : "attitude, report", nullptr, [mode](RuntimeConfig& c) {
                           protect(c, mode);
                           assign(c, 0, AttitudeCommand{0.2, 0.05, kHold, 0.785, kHold, 55.0}, kLegacyAxes);
                       }});
        out.push_back({limit ? "acceleration, limit" : "acceleration, report", nullptr, [mode](RuntimeConfig& c) {
                           protect(c, mode);
                           assign(c, 0, AccelerationCommand{1.5, 0.2, kHold, 0.6}, kLegacyAxes);
                       }});
        out.push_back({limit ? "velocity, limit" : "velocity, report", nullptr, [mode](RuntimeConfig& c) {
                           protect(c, mode);
                           assign(c, 0, VelocityCommand{60.0, 2.0, 0.5, kHold}, kLegacyAxes);
                       }});
        out.push_back({limit ? "actuator, limit" : "actuator, report", nullptr, [mode](RuntimeConfig& c) {
                           protect(c, mode);
                           assign(c, 0, ActuatorCommand{0.1, -0.05, 0.0, 0.6}, kLegacyAxes);
                       }});
    }
    return out;
}

/// A slowly varying synthetic flight, so the loops see changing errors.
void advance(sim::VehicleState& s, double dt) {
    s.simTime += dt;
    const double t = s.simTime;
    s.eulerRad[0] = 0.1 * std::sin(0.5 * t);
    s.eulerRad[1] = 0.03 * std::cos(0.3 * t);
    s.angularRateBodyRadS[0] = 0.05 * std::cos(0.5 * t);
    s.angularRateBodyRadS[1] = -0.009 * std::sin(0.3 * t);
    s.airspeedTrueMs = s.airspeedCalibratedMs = 55.0 + 2.0 * std::sin(0.1 * t);
    s.velocityNedMs[2] = -1.0 * std::sin(0.2 * t);
    s.loadFactor = 1.0 + 0.1 * std::sin(0.7 * t);
    s.alphaRad = 0.05 + 0.01 * std::sin(0.7 * t);
}

struct StackRun {
    sim::VehicleState state = levelFlight();
    Rng rng{3};
    ControlStack stack;
    sim::ControlInputs out;
    double dt = 1.0 / 120.0;
    explicit StackRun(const Case& c) {
        if (c.attitudeLoop) stack.use(Level::Attitude, c.attitudeLoop);
        if (c.configure) c.configure(stack.config());
        else stack.command(c.command(state));
    }
    void run(int n) {
        for (int i = 0; i < n; ++i) {
            advance(state, dt);
            ControlContext ctx{1, state, state, dt, nullptr, &rng};
            stack.update(ctx, out);
        }
    }
};

double seconds(std::chrono::steady_clock::duration d) { return std::chrono::duration<double>(d).count(); }

// --- modes -------------------------------------------------------------------------------

int micro() {
    std::printf("%-14s %10s\n", "case", "ns/update");
    for (const auto& c : cases()) {
        StackRun r(c);
        r.run(10000);
        std::vector<double> ns;
        for (int rep = 0; rep < 5; ++rep) {
            const auto t0 = std::chrono::steady_clock::now();
            r.run(200000);
            ns.push_back(seconds(std::chrono::steady_clock::now() - t0) / 200000 * 1e9);
        }
        std::sort(ns.begin(), ns.end());
        std::printf("%-14s %10.1f\n", c.name, ns[2]);
    }
    return 0;
}

session::WorldOptions benchWorld(const char* name, unsigned workers) {
    auto o = flights::worldOptions(name, FSIM_TEST_JSBSIM_ROOT, workers);
    o.pinWorkers = true;
    return o;
}

std::vector<std::uint32_t> spawn(session::World& w, const char* type, int n, double altitudeM, double tasMs) {
    std::vector<std::uint32_t> ids;
    for (int i = 0; i < n; ++i) {
        const auto id = w.createVehicle(flights::spec(std::string(type) + "-" + std::to_string(i), type, i, altitudeM, tasMs));
        if (id) ids.push_back(id);
    }
    return ids;
}

int command() {
    session::World w(benchWorld("bench-command", 1));
    const auto ids = spawn(w, "jsbsim:c172x", 64, 1500.0, 55.0);
    const AttitudeCommand att{0.2, 0.05, kHold, 0.785, kHold, 55.0};
    const VelocityCommand vel{55.0, 1.0, kHold, 0.02};
    auto time = [&](const char* name, const std::function<void(int)>& call, int reps) {
        call(0);
        const auto t0 = std::chrono::steady_clock::now();
        for (int k = 1; k <= reps; ++k) call(k);
        const double ns = seconds(std::chrono::steady_clock::now() - t0) / (static_cast<double>(reps) * static_cast<double>(ids.size())) * 1e9;
        std::printf("%-26s %8.1f ns/call\n", name, ns);
    };
    std::printf("%zu vehicles\n", ids.size());
    time("same level (update)", [&](int) { for (auto id : ids) w.command(id, att); }, 20000);
    time("level switch (new)", [&](int k) { for (auto id : ids) (k % 2) ? w.command(id, vel) : w.command(id, att); }, 20000);
    const auto hold = flights::behavior("hold");
    time("behaviour (new)", [&](int) { for (auto id : ids) w.command(id, hold); }, 2000);
    // the contract layer's own path: an activity per vehicle, its setpoint checked and updated
    std::vector<ActivityId> activities;
    for (auto id : ids) activities.push_back(w.submit(id, att).activity);
    time("update, checked", [&](int) { for (auto a : activities) w.update(a, att); }, 20000);
    return 0;
}

/// `off`: protection off. `classic`: the designs' attitude flown by pid_attitude
/// straight to the actuators, as before step 5b, not over pseudo-controls.
int world(bool protectionOff, bool classic) {
    struct Setup {
        const char* type;
        int count;
        double altitude, tas;
    };
    // the World's default (physical cores - 2), pinned
    const unsigned physical = platform::physicalCoreCount();
    const unsigned workers = physical > 2 ? physical - 2 : 1u;
    std::printf("%u workers, pinned\n", workers);
    for (const Setup& s : {Setup{"jsbsim:c172x", 64, 1500.0, 55.0}, Setup{"jsbsim:f16c", 32, 3000.0, 160.0}, Setup{"jsbsim:b52h", 32, 3000.0, 180.0}}) {
        session::World w(benchWorld("bench-world", workers));
        const auto ids = spawn(w, s.type, s.count, s.altitude, s.tas);
        if (ids.empty()) {
            std::printf("%-14s not found\n", s.type);
            continue;
        }
        if (protectionOff)
            for (auto id : ids) w.setProtection(id, ProtectionMode::Off);
        if (classic)
            for (auto id : ids) w.controls(id)->use(Level::Attitude, "pid_attitude");
        double best = 0.0;
        for (int rep = 0; rep < 3; ++rep) {
            const auto steps0 = w.vehicleSteps();
            const auto t0 = std::chrono::steady_clock::now();
            for (int k = 0; k < 1000; ++k) {
                const double t = k / 30.0;
                for (std::size_t i = 0; i < ids.size(); ++i)
                    w.command(ids[i], AttitudeCommand{0.2 * std::sin(0.3 * t + static_cast<double>(i)), 0.03, kHold, 0.785, kHold, s.tas});
                w.step();
            }
            best = std::max(best, static_cast<double>(w.vehicleSteps() - steps0) / seconds(std::chrono::steady_clock::now() - t0));
        }
        std::printf("%-14s %4zu vehicles %12.0f vehicle-steps/s\n", s.type, ids.size(), best);
    }
    return 0;
}

int alloc() {
    if (!installAllocationCounter()) return 2;
    int failures = 0;
    std::printf("%-26s %12s\n", "case", "allocations");
    for (const auto& c : cases()) {
        StackRun r(c);
        r.run(1000);
        std::uint64_t n;
        {
            Counting counting;
            r.run(10000);
            n = counting.count();
        }
        std::printf("update: %-18s %12llu\n", c.name, static_cast<unsigned long long>(n));
        failures += n != 0;
    }
    // The world, every vehicle commanded every step (as VecEnv does) at each
    // level, then a behaviour: the platform's own allocations must be none;
    // JSBSim's, made as it steps, are shown for reference.
    session::World w(benchWorld("bench-alloc", 4));
    const auto ids = spawn(w, "jsbsim:c172x", 16, 1500.0, 55.0);
    const auto hold = flights::behavior("hold");
    auto loiter = flights::behavior("loiter");
    loiter.params = {{"radius_m", 800.0}, {"altitude_m", 1600.0}};
    struct WorldCase {
        const char* name;
        std::function<void(std::uint32_t, int)> command;
    };
    const std::vector<WorldCase> worldCases = {
        {"actuator each step", [&](std::uint32_t id, int k) { w.command(id, ActuatorCommand{0.01 * std::sin(k * 0.1), -0.02, 0.0, 0.7}); }},
        {"attitude each step", [&](std::uint32_t id, int k) { w.command(id, AttitudeCommand{0.1 * std::sin(k * 0.1), 0.03, kHold, 0.785, kHold, 55.0}); }},
        {"velocity each step", [&](std::uint32_t id, int k) { w.command(id, VelocityCommand{55.0, std::sin(k * 0.1), kHold, 0.02}); }},
        {"update each step", [&](std::uint32_t id, int k) {
             static std::vector<ActivityId> activity(64, 0);
             if (k == 0) activity[id] = w.submit(id, AttitudeCommand{0.0, 0.03, kHold, 0.785, kHold, 55.0}).activity;
             else w.update(activity[id], AttitudeCommand{0.1 * std::sin(k * 0.1), 0.03, kHold, 0.785, kHold, 55.0});
         }},
        {"flaps update each step", [&](std::uint32_t id, int k) {
             static std::vector<ActivityId> activity(64, 0);
             if (k == 0) activity[id] = w.submit(id, FlapsCommand{0.1}).activity;
             else w.update(activity[id], FlapsCommand{0.1 + 0.001 * (k % 10)});
         }},
        {"hold once", [&](std::uint32_t id, int k) { if (k == 0) w.command(id, hold); }},
        {"loiter once", [&](std::uint32_t id, int k) { if (k == 0) w.command(id, loiter); }},
        // step 3: an autopilot on pitch and thrust, the policy's bank updated every step
        {"axes apart each step", [&](std::uint32_t id, int k) {
             static std::vector<ActivityId> activity(64, 0);
             CommandOptions lateral;
             lateral.axes = kLateral;
             const AttitudeCommand bank{0.2 * std::sin(k * 0.1), kHold, kHold, 0.785, kHold, kHold};
             if (k == 0) {
                 CommandOptions autopilot;
                 autopilot.source = Source::Autopilot;
                 autopilot.axes = axisBit(Axis::Pitch) | axisBit(Axis::Thrust);
                 const bool held = w.submit(id, VelocityCommand{55.0, 0.0, kHold, kHold}, autopilot).accepted();
                 activity[id] = w.submit(id, bank, lateral).activity;
                 if (!held || !activity[id]) std::fprintf(stderr, "axes apart: vehicle %u refused\n", id), std::exit(3);
             } else if (!w.update(activity[id], bank).accepted()) {
                 std::fprintf(stderr, "axes apart: update refused\n"), std::exit(3);
             }
         }},
        // everything let go: the vehicle default's hold flies it
        {"default hold", [&](std::uint32_t id, int k) {
             if (k != 0) return;
             for (const auto& a : w.activities(id))
                 if (a.live()) w.cancel(a.id);
             if (w.setVehicleDefault(id, VehicleDefault::Hold) != Reason::None || w.controls(id)->activeLevel() != Level::Velocity)
                 std::fprintf(stderr, "default hold: vehicle %u refused\n", id), std::exit(3);
         }},
    };
    for (const auto& wc : worldCases) {
        for (int k = 0; k < 50; ++k) {
            for (auto id : ids) wc.command(id, k);
            w.step();
        }
        std::uint64_t n, fm;
        {
            Counting counting;
            for (int k = 50; k < 150; ++k) {
                for (auto id : ids) wc.command(id, k);
                w.step();
            }
            n = counting.count();
            fm = counting.flightModel();
        }
        std::printf("world: %-19s %12llu   (JSBSim: %llu)\n", wc.name, static_cast<unsigned long long>(n), static_cast<unsigned long long>(fm));
        failures += n != 0;
    }
    {
        // envelope protection in the world: a design with an envelope, its demand limited every step
        session::World pw(benchWorld("bench-alloc-protection", 4));
        const auto designed = spawn(pw, "jsbsim:c172", 8, 1500.0, 45.0);
        auto pull = [&](int k) {
            for (auto id : designed) pw.command(id, AccelerationCommand{2.5 + 0.1 * std::sin(k * 0.1), 0.1, kHold, 0.8});
        };
        for (int k = 0; k < 50; ++k) pull(k), pw.step();
        std::uint64_t n, fm;
        std::uint32_t limited = 0;
        {
            Counting counting;
            for (int k = 50; k < 150; ++k) pull(k), pw.step();
            n = counting.count();
            fm = counting.flightModel();
        }
        for (auto id : designed) limited += pw.envelope(id)[Limit::AlphaMax].limitedUpdates;
        std::printf("world: %-19s %12llu   (JSBSim: %llu; %u limited)\n", "limit each step", static_cast<unsigned long long>(n),
                    static_cast<unsigned long long>(fm), limited);
        failures += n != 0;
        if (designed.empty() || limited == 0) {
            std::printf("FAIL: protection never limited (%zu vehicles)\n", designed.size());
            ++failures;
        }
    }
    std::printf(failures ? "FAIL: %d case(s) allocate\n" : "no allocations\n", failures);
    return failures ? 1 : 0;
}

int digest(const char* path, bool protectionOff) {
    session::World w(flights::worldOptions("bench-digest", FSIM_TEST_JSBSIM_ROOT, 3));
    flights::Flights f(w, true);
    // "off": every vehicle without envelope protection, as before step 4 (the hangar designs have an envelope)
    if (protectionOff)
        for (const auto& flight : f.flights()) w.setProtection(flight.id, ProtectionMode::Off);
    std::vector<flights::Digest> digests(f.flights().size());
    for (int k = 0; k < 900; ++k) {
        f.drive(k);
        w.step();
        for (std::size_t i = 0; i < f.flights().size(); ++i) {
            const auto id = f.flights()[i].id;
            digests[i].add(*w.vehicleState(id), *w.inputs(id));
        }
    }
    std::ofstream out(path, std::ios::binary); // LF, as the repository keeps text
    for (std::size_t i = 0; i < f.flights().size(); ++i) {
        const auto& s = *w.vehicleState(f.flights()[i].id);
        char line[256];
        std::snprintf(line, sizeof line, "%-18s %016llx  alt %9.3f  lat %.9f  lon %.9f\n", f.flights()[i].name.c_str(),
                      static_cast<unsigned long long>(digests[i].value()), s.altitudeMslM, s.latitudeRad / flights::kDeg,
                      s.longitudeRad / flights::kDeg);
        out << line;
        std::fputs(line, stdout);
    }
    return 0;
}

int checkpoints(const char* path) {
    session::World w(flights::worldOptions("bench-checkpoints", FSIM_TEST_JSBSIM_ROOT, 1));
    flights::Flights f(w, false);
    std::ofstream out(path, std::ios::binary); // LF, as the repository keeps text
    out << "# The stock c172x's closed-loop flights of tests/control_flights.h through the legacy command path:\n"
           "# name, world step, then latitude, longitude (rad), altitude (m), roll, pitch, yaw (rad),\n"
           "# true airspeed (m/s), aileron, elevator, rudder, throttle. Recorded before the control\n"
           "# architecture changed (docs/control-architecture.md, section 12.4); a real change moves\n"
           "# them far more than a toolchain's last bits.\n";
    for (int k = 0; k < flights::kCheckpointSteps; ++k) {
        f.drive(k);
        w.step();
        if ((k + 1) % flights::kCheckpointEvery) continue;
        for (const auto& fl : f.flights()) {
            if (!fl.closedLoop) continue;
            out << fl.name << ' ' << (k + 1);
            char buf[40];
            for (double v : flights::checkpoint(*w.vehicleState(fl.id), *w.inputs(fl.id))) {
                std::snprintf(buf, sizeof buf, " %.17g", v);
                out << buf;
            }
            out << '\n';
        }
    }
    std::printf("wrote %s\n", path);
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    fsim::log::setLevel(fsim::log::Level::Warn);
    const std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "micro") return micro();
    if (mode == "command") return command();
    if (mode == "world") return world(argc > 2 && std::string(argv[2]) == "off", argc > 2 && std::string(argv[2]) == "classic");
    if (mode == "alloc") return alloc();
    if (mode == "digest" && argc > 2) return digest(argv[2], argc > 3 && std::string(argv[3]) == "off");
    if (mode == "checkpoints" && argc > 2) return checkpoints(argv[2]);
    std::fprintf(stderr, "usage: fsim_control_bench micro | command | world [off | classic] | alloc | digest FILE [off] | checkpoints FILE\n");
    return 2;
}
