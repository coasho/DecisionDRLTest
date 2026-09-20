# Environment, effects and communication

## Environment (real-time)

`world.environment()` is the world's global conditions. Every setter takes
effect at the next `step()` and may be called every step; changes are a few
property writes per vehicle. The viewer lights the scene from the world's
clock and shows the wind and weather.

```cpp
auto& env = world.environment();
env.setTime(fsim::Utc{2026, 6, 21, 5, 30, 0});       // sim time starts here; sun and sky follow
env.setTime(1782000000.0);                            // or Unix seconds
env.setWind({.directionDeg = 240, .speedMs = 12, .gustMs = 4, .turbulence = 0.4}); // FROM 240 deg
env.setAtmosphere({.temperatureSeaLevelK = 305, .pressureSeaLevelPa = 100200, .humidity = 0.7});
env.setWeather({.visibilityM = 8000, .cloudBaseM = 900, .cloudCover = 0.8, .precipitation = 0.3});
env.setTimeFactor(4.0);                               // informational hint for viewers; you set the pace
double utc = env.utcSeconds();                        // epoch + simulation time
```

| Parameter | Effect on the flight models | Effect in the viewer |
| --- | --- | --- |
| Time | available to behaviours/effects (`ctx.world->simTime()`) | sun position, sky |
| Wind, turbulence | JSBSim steady wind (NED), Milspec/Dryden turbulence with the vehicle's seed | shown in the monitor |
| Atmosphere | JSBSim sea-level temperature and pressure (density, speed of sound, engine power) | monitor |
| Weather | none (sensor/visual models) | monitor (rendering effects later) |

`env.set(EnvironmentState)` writes every field at once, `env.state()` reads them.

## Effects

`#include <fsim/Effects.h>`, `<fsim/BuiltinEffects.h>`

An **effect** is attached to a vehicle (or to every vehicle with
`world.addEffectToAll<E>()`) and runs once per FDM step before the flight
model. It sees the truth state and may touch four channels through
`EffectContext`:

| Channel | Calls | Where it lands |
| --- | --- | --- |
| Local wind | `addWindNed(n, e, d)`, `setWindNed(...)` | the vehicle's JSBSim wind (on top of, or replacing, the global wind) |
| Force / moment | `addForceBody(x, y, z)` (N), `addMomentBody(l, m, n)` (N m) | a generic external reaction at the centre of gravity, injected into every stock aircraft at load |
| Properties | `property("path").set(v)` | any flight-model property: mass, fuel, engine health, surface limits |
| Sensors | `sensed.state`, `sensed.gnssValid`, `sensed.airDataValid`, `sensed.positionErrorM` | what `vehicle.sensed()` and the built-in control loops see; truth is untouched |

Built-ins (`fsim::effects`):

| Class / id | Parameters |
| --- | --- |
| `GaussianSensorNoise` / `gaussian_sensor_noise` | `positionSigmaM` 3, `altitudeSigmaM` 2, `velocitySigmaMs` 0.3, `attitudeSigmaRad` 0.005, `rateSigmaRadS` 0.01, `airspeedSigmaMs` 0.5 |
| `SensorLatency` / `sensor_latency` | `delaySteps` 6 (FDM steps) |
| `ConstantForce` / `constant_force` | `forceN[3]`, `momentNm[3]` (body frame) |
| `WindGusts` / `wind_gusts` | `meanIntervalS` 20, `peakMs` 6, `durationS` 4 (random 1-cosine gusts) |
| `GnssDegradation` / `gnss_degradation` | `lossProbabilityPerS` 0.01, `outageS` 5, `driftM` 10 |

```cpp
auto* noise = v.addEffect<fsim::effects::GaussianSensorNoise>();
noise->positionSigmaM = 8.0;
v.addEffect<fsim::effects::SensorLatency>(12u);
world.addEffectToAll<fsim::effects::WindGusts>();

struct Icing final : fsim::effects::Effect {          // your own: e.g. drag and weight creep
    const char* id() const noexcept override { return "icing"; }
    void apply(fsim::effects::EffectContext& ctx) override {
        const double v = ctx.state.airspeedTrueMs;
        ctx.addForceBody(-0.02 * v * v, 0.0, 0.0);      // extra drag along -x
        ctx.property("inertia/pointmass-weight-lbs[0]").set(40.0);
    }
};
v.addEffect(std::make_unique<Icing>());
```

Effects keep their state across steps, get `onReset()` on a vehicle reset,
and can be switched with `enabled`. They run on the worker thread that owns
the vehicle: touch only your own members and the context.

## Communication

`#include <fsim/Comm.h>`

Vehicles and external nodes exchange messages through the world's
`comm::Network`, stepped once per world step. Everything is an interface with
a small built-in implementation:

| Piece | Built-ins | Extend by |
| --- | --- | --- |
| `Node` | one per vehicle (address = vehicle id); `network.createNode(address)` for ground stations or the trainer | - |
| `Message` | `from`, `to` (address or `kBroadcast`), `channel`, `timeSent`, `timeDelivered`, `Payload{format, bytes}` | - |
| Codecs | `encodeRaw<T>` / `decodeRaw<T>` (trivially copyable structs), `JsonCodec` for named numeric `Fields` | `comm::Codec` with your own format id (>= `kUserFormat`) |
| `Medium` | `IdealMedium` (instant, lossless), `LinkModel` (`rangeM`, `latencyS`, `jitterS`, `lossProbability`) | subclass `Medium::route()`; bridges to real transports fit here |
| `Protocol` | `BeaconProtocol(periodS, channel)`: periodic JSON state reports | subclass `Protocol` (`onStep`, `onReceive`) |

```cpp
auto& net = world.network();
net.setMedium(std::make_unique<fsim::comm::LinkModel>());           // range/latency/loss
net.attach(red.id(), std::make_unique<fsim::comm::BeaconProtocol>(1.0, 1));

fsim::comm::Message m;
m.from = blue.id(); m.to = red.id(); m.channel = 7;
m.payload = fsim::comm::JsonCodec::encode({{"cmd", 2}, {"alt_m", 2500}});
net.send(std::move(m));

world.step();
for (const auto& msg : red.node()->inbox()) {                      // delivered during this step
    fsim::comm::Fields f;
    if (fsim::comm::JsonCodec::decode(msg.payload, f)) /* ... */;
}
```

Messages sent during a step are routed by the medium at the next
`network.step()` (inside `world.step()`), delivered when due, and stay in the
receiver's `inbox()` until the following step. The `LinkModel` uses the
vehicles' true positions for range checks and the world's random stream for
loss and jitter, so results stay deterministic.
