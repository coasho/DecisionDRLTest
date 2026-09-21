// Smoke test of an installed fsim package: a world, a vehicle, a command, a step.
#include <fsim/World.h>
#ifdef HAVE_FSIM_VISION
#include <fsim/Vision.h>
#endif

#include <cstdio>

int main() {
    fsim::WorldOptions o;
    o.name = "consumer";
    o.publish = false;
    fsim::World world(o);
    fsim::VehicleSpec s;
    s.name = "one";
    s.initial.altitudeMslM = 1500.0;
    s.initial.airspeedTrueMs = 60.0;
    fsim::Vehicle v = world.createVehicle(s);
    fsim::control::AttitudeCommand a;
    a.pitchRad = 0.03;
    v.command(a);
    world.step(30);
    std::printf("fsim %s: %s at %.1f m after %.2f s\n", fsim::version(), v.name().c_str(), v.state().altitudeMslM, world.time());
#ifdef HAVE_FSIM_VISION
    std::printf("vision available\n");
#endif
    return v.state().altitudeMslM > 1000.0 ? 0 : 1;
}
