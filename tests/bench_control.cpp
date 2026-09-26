// fsim_control_bench: the control architecture's measurements
// (docs/control-architecture.md, section 12).
//
//   fsim_control_bench micro          ControlStack::update, ns per update, per level and behaviour
//   fsim_control_bench command        World::command (the legacy path), ns per call
//   fsim_control_bench world          vehicle-steps/s with every vehicle commanded every step
//   fsim_control_bench alloc          heap allocations in the steady state (exit 1 if any)
//   fsim_control_bench digest FILE    per-flight digests of every world step (compare across builds)
//   fsim_control_bench checkpoints FILE   the stock c172x checkpoints (tests/data)
//
// Heap allocations are counted by redirecting the malloc, calloc and realloc
// imports of this executable and of libstdc++ (whose operator new calls
// malloc), so an allocation made anywhere in the platform's code - inlined
// templates or the library's own string members - is seen.

#include "control_flights.h"

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
};

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
    explicit StackRun(const Case& c) { stack.command(c.command(state)); }
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

int world() {
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
    std::printf(failures ? "FAIL: %d case(s) allocate\n" : "no allocations\n", failures);
    return failures ? 1 : 0;
}

int digest(const char* path) {
    session::World w(flights::worldOptions("bench-digest", FSIM_TEST_JSBSIM_ROOT, 3));
    flights::Flights f(w, true);
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
    if (mode == "world") return world();
    if (mode == "alloc") return alloc();
    if (mode == "digest" && argc > 2) return digest(argv[2]);
    if (mode == "checkpoints" && argc > 2) return checkpoints(argv[2]);
    std::fprintf(stderr, "usage: fsim_control_bench micro | command | world | alloc | digest FILE | checkpoints FILE\n");
    return 2;
}
