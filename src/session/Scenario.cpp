// Scenario files (design 9.13): parse, dump and apply to a session::World.

#include "fsim/Scenario.h"

#include "fsim/BuiltinEffects.h"

#include "core/Geodesy.h"
#include "core/Json.h"
#include "core/Log.h"
#include "core/Time.h"
#include "session/Scenario.h"
#include "session/World.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace fsim {

using core::Json;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

double deg(const Json& o, const char* key, double fallbackRad) {
    const Json* v = o.find(key);
    return v && v->isNumber() ? v->asNumber() * kDeg : fallbackRad;
}

/// ISO 8601 "YYYY-MM-DD[Thh:mm[:ss[.fff]]][Z]" (UTC only).
bool parseIsoUtc(const std::string& text, Utc& out) {
    int y = 0, mo = 0, d = 0, h = 0, mi = 0;
    double s = 0.0;
    const int n = std::sscanf(text.c_str(), "%d-%d-%dT%d:%d:%lf", &y, &mo, &d, &h, &mi, &s);
    if (n < 3) return false;
    out.year = y; out.month = mo; out.day = d;
    out.hour = n >= 4 ? h : 0;
    out.minute = n >= 5 ? mi : 0;
    out.second = n >= 6 ? s : 0.0;
    return true;
}

void parseEffects(const Json& list, std::vector<EffectSpec>& out, const std::string& where) {
    if (list.isNull()) return;
    if (!list.isArray()) throw Error(where + ": 'effects' must be an array");
    for (const auto& e : list.asArray()) {
        if (!e.isObject()) throw Error(where + ": each effect must be an object");
        EffectSpec spec;
        spec.id = e.string("id", "");
        if (spec.id.empty()) throw Error(where + ": effect without 'id'");
        for (const auto& [k, v] : e.asObject()) {
            if (k == "id") continue;
            if (!v.isNumber()) throw Error(where + ": effect '" + spec.id + "' parameter '" + k + "' must be a number");
            spec.params.emplace_back(k, v.asNumber());
        }
        out.push_back(std::move(spec));
    }
}

control::Command parseCommand(const Json& c, const std::string& where, std::string& targetName) {
    const std::string level = c.string("level", "");
    if (level == "actuator") {
        control::ActuatorCommand a;
        a.aileron = c.number("aileron", a.aileron);
        a.elevator = c.number("elevator", a.elevator);
        a.rudder = c.number("rudder", a.rudder);
        a.throttle = c.number("throttle", a.throttle);
        a.flaps = c.number("flaps", a.flaps);
        a.gearDown = c.number("gear_down", a.gearDown);
        a.brakeLeft = c.number("brake_left", a.brakeLeft);
        a.brakeRight = c.number("brake_right", a.brakeRight);
        return a;
    }
    if (level == "attitude") {
        control::AttitudeCommand a;
        a.rollRad = deg(c, "roll_deg", a.rollRad);
        a.pitchRad = deg(c, "pitch_deg", a.pitchRad);
        a.headingRad = deg(c, "heading_deg", a.headingRad);
        a.maxBankRad = deg(c, "max_bank_deg", a.maxBankRad);
        a.throttle = c.number("throttle", a.throttle);
        a.airspeedMs = c.number("airspeed_ms", a.airspeedMs);
        return a;
    }
    if (level == "acceleration") {
        control::AccelerationCommand a;
        a.loadFactorG = c.number("load_factor_g", a.loadFactorG);
        a.rollRateRadS = deg(c, "roll_rate_deg_s", a.rollRateRadS);
        a.longitudinalMs2 = c.number("longitudinal_ms2", a.longitudinalMs2);
        a.throttle = c.number("throttle", a.throttle);
        return a;
    }
    if (level == "velocity") {
        control::VelocityCommand v;
        v.airspeedMs = c.number("airspeed_ms", v.airspeedMs);
        v.verticalSpeedMs = c.number("vertical_speed_ms", v.verticalSpeedMs);
        v.headingRad = deg(c, "heading_deg", v.headingRad);
        v.turnRateRadS = deg(c, "turn_rate_deg_s", v.turnRateRadS);
        return v;
    }
    auto point = [&](const Json& p) {
        control::PositionCommand pc;
        if (!p.has("lat_deg") || !p.has("lon_deg")) throw Error(where + ": a position needs 'lat_deg' and 'lon_deg'");
        pc.latitudeRad = deg(p, "lat_deg", 0.0);
        pc.longitudeRad = deg(p, "lon_deg", 0.0);
        pc.altitudeMslM = p.number("alt_msl_m", pc.altitudeMslM);
        pc.airspeedMs = p.number("airspeed_ms", pc.airspeedMs);
        pc.captureRadiusM = p.number("capture_radius_m", pc.captureRadiusM);
        return pc;
    };
    if (level == "position") return point(c);
    if (level == "behavior" || level == "behaviour") {
        control::BehaviorCommand b;
        b.id = c.string("id", "");
        if (b.id.empty()) throw Error(where + ": a behavior command needs 'id'");
        targetName = c.string("target", "");
        const Json& params = c.child("params");
        if (!params.isNull()) {
            if (!params.isObject()) throw Error(where + ": 'params' must be an object");
            for (const auto& [k, v] : params.asObject()) {
                if (!v.isNumber()) throw Error(where + ": behaviour parameter '" + k + "' must be a number");
                b.params[k] = v.asNumber();
            }
        }
        const Json& points = c.child("points");
        if (!points.isNull()) {
            if (!points.isArray()) throw Error(where + ": 'points' must be an array");
            for (const auto& p : points.asArray()) b.points.push_back(point(p));
        }
        return b;
    }
    throw Error(where + ": unknown command level '" + level + "'");
}

Json dumpCommand(const control::Command& command, const std::string& targetName) {
    Json c;
    auto num = [&](const char* key, double v) { if (!std::isnan(v)) c.set(key, v); };
    auto degOut = [&](const char* key, double rad) { if (!std::isnan(rad)) c.set(key, rad / kDeg); };
    switch (control::levelOf(command)) {
    case control::Level::Actuator: {
        const auto& a = std::get<control::ActuatorCommand>(command);
        c.set("level", "actuator");
        num("aileron", a.aileron); num("elevator", a.elevator); num("rudder", a.rudder); num("throttle", a.throttle);
        num("flaps", a.flaps); num("gear_down", a.gearDown); num("brake_left", a.brakeLeft); num("brake_right", a.brakeRight);
        break;
    }
    case control::Level::Attitude: {
        const auto& a = std::get<control::AttitudeCommand>(command);
        c.set("level", "attitude");
        degOut("roll_deg", a.rollRad); degOut("pitch_deg", a.pitchRad); degOut("heading_deg", a.headingRad);
        degOut("max_bank_deg", a.maxBankRad); num("throttle", a.throttle); num("airspeed_ms", a.airspeedMs);
        break;
    }
    case control::Level::Acceleration: {
        const auto& a = std::get<control::AccelerationCommand>(command);
        c.set("level", "acceleration");
        num("load_factor_g", a.loadFactorG); degOut("roll_rate_deg_s", a.rollRateRadS); num("longitudinal_ms2", a.longitudinalMs2); num("throttle", a.throttle);
        break;
    }
    case control::Level::Velocity: {
        const auto& v = std::get<control::VelocityCommand>(command);
        c.set("level", "velocity");
        num("airspeed_ms", v.airspeedMs); num("vertical_speed_ms", v.verticalSpeedMs); degOut("heading_deg", v.headingRad); degOut("turn_rate_deg_s", v.turnRateRadS);
        break;
    }
    case control::Level::Position: {
        const auto& p = std::get<control::PositionCommand>(command);
        c.set("level", "position");
        degOut("lat_deg", p.latitudeRad); degOut("lon_deg", p.longitudeRad); num("alt_msl_m", p.altitudeMslM); num("airspeed_ms", p.airspeedMs); num("capture_radius_m", p.captureRadiusM);
        break;
    }
    case control::Level::Behavior: {
        const auto& b = std::get<control::BehaviorCommand>(command);
        c.set("level", "behavior");
        c.set("id", b.id);
        if (!targetName.empty()) c.set("target", targetName);
        if (!b.params.empty()) {
            Json params;
            for (const auto& [k, v] : b.params) params.set(k, v);
            c.set("params", std::move(params));
        }
        if (!b.points.empty()) {
            Json points;
            for (const auto& p : b.points) {
                Json pt;
                pt.set("lat_deg", p.latitudeRad / kDeg).set("lon_deg", p.longitudeRad / kDeg).set("alt_msl_m", p.altitudeMslM);
                if (!std::isnan(p.airspeedMs)) pt.set("airspeed_ms", p.airspeedMs);
                pt.set("capture_radius_m", p.captureRadiusM);
                points.push(std::move(pt));
            }
            c.set("points", std::move(points));
        }
        break;
    }
    default: break;
    }
    return c;
}

Json dumpEffects(const std::vector<EffectSpec>& effects) {
    Json list;
    for (const auto& e : effects) {
        Json o;
        o.set("id", e.id);
        for (const auto& [k, v] : e.params) o.set(k, v);
        list.push(std::move(o));
    }
    return list;
}

} // namespace

Scenario parseScenario(std::string_view json, std::string_view source) {
    Scenario sc;
    sc.source = std::string(source);
    Json doc;
    try {
        doc = Json::parse(json, source);
    } catch (const std::exception& e) {
        throw Error(e.what());
    }
    if (!doc.isObject()) throw Error(sc.source + ": the document must be an object");
    try {
        const Json& w = doc.child("world");
        if (!w.isNull()) {
            if (!w.isObject()) throw Error("'world' must be an object");
            auto& o = sc.world;
            o.name = w.string("name", o.name);
            o.dt = w.number("dt", o.dt);
            o.frameSkip = static_cast<int>(w.number("frame_skip", o.frameSkip));
            o.workers = static_cast<unsigned>(w.number("workers", o.workers));
            o.pinWorkers = w.boolean("pin_workers", o.pinWorkers);
            o.seed = static_cast<std::uint64_t>(w.number("seed", static_cast<double>(o.seed)));
            o.capacity = static_cast<std::uint32_t>(w.number("capacity", o.capacity));
            o.publish = w.boolean("publish", o.publish);
            o.publishIntervalSeconds = w.number("publish_interval_s", o.publishIntervalSeconds);
            o.jsbsimRoot = w.string("jsbsim_root", o.jsbsimRoot);
            o.terrain = w.boolean("terrain", o.terrain);
            o.terrainUrl = w.string("terrain_url", o.terrainUrl);
            o.terrainZoom = static_cast<unsigned>(w.number("terrain_zoom", o.terrainZoom));
            o.recordPath = w.string("record_path", o.recordPath);
            o.recordIntervalSeconds = w.number("record_interval_s", o.recordIntervalSeconds);
            if (o.dt <= 0.0 || o.frameSkip <= 0) throw Error("'world.dt' and 'world.frame_skip' must be positive");
        }

        const Json& env = doc.child("environment");
        if (!env.isNull()) {
            if (!env.isObject()) throw Error("'environment' must be an object");
            sc.hasEnvironment = true;
            auto& e = sc.environment;
            if (const Json* utc = env.find("utc")) {
                Utc u;
                if (!utc->isString() || !parseIsoUtc(utc->asString(), u)) throw Error("'environment.utc' must be an ISO 8601 UTC time like \"2026-06-21T10:30:00Z\"");
                e.epochUtcSeconds = core::unixSecondsFromCivil(u.year, u.month, u.day, u.hour, u.minute, u.second);
            }
            e.epochUtcSeconds = env.number("unix_seconds", e.epochUtcSeconds);
            e.timeFactor = env.number("time_factor", e.timeFactor);
            const Json& wind = env.child("wind");
            e.windDirectionDeg = wind.number("direction_deg", e.windDirectionDeg);
            e.windSpeedMs = wind.number("speed_ms", e.windSpeedMs);
            e.windGustMs = wind.number("gust_ms", e.windGustMs);
            e.turbulence = wind.number("turbulence", e.turbulence);
            const Json& atm = env.child("atmosphere");
            e.temperatureSeaLevelK = atm.number("temperature_sea_level_k", e.temperatureSeaLevelK);
            e.pressureSeaLevelPa = atm.number("pressure_sea_level_pa", e.pressureSeaLevelPa);
            e.humidity = atm.number("humidity", e.humidity);
            const Json& wx = env.child("weather");
            e.visibilityM = wx.number("visibility_m", e.visibilityM);
            e.cloudBaseM = wx.number("cloud_base_m", e.cloudBaseM);
            e.cloudCover = wx.number("cloud_cover", e.cloudCover);
            e.precipitation = wx.number("precipitation", e.precipitation);
        }

        parseEffects(doc.child("effects"), sc.effects, "effects");

        const Json& vehicles = doc.child("vehicles");
        if (!vehicles.isNull()) {
            if (!vehicles.isArray()) throw Error("'vehicles' must be an array");
            for (const auto& v : vehicles.asArray()) {
                if (!v.isObject()) throw Error("each vehicle must be an object");
                ScenarioVehicle sv;
                sv.spec.name = v.string("name", "");
                if (sv.spec.name.empty()) throw Error("vehicle without 'name'");
                const std::string where = "vehicle '" + sv.spec.name + "'";
                sv.spec.type = v.string("type", sv.spec.type);
                sv.spec.model = v.string("model", sv.spec.model);
                sv.spec.controlDivider = static_cast<unsigned>(v.number("control_divider", sv.spec.controlDivider));
                sv.count = static_cast<unsigned>(v.number("count", sv.count));
                sv.spacingM = v.number("spacing_m", sv.spacingM);
                if (sv.count == 0) throw Error(where + ": 'count' must be at least 1");
                const Json& ic = v.child("initial");
                auto& i = sv.spec.initial;
                i.latitudeDeg = ic.number("lat_deg", i.latitudeDeg);
                i.longitudeDeg = ic.number("lon_deg", i.longitudeDeg);
                i.altitudeMslM = ic.number("alt_msl_m", i.altitudeMslM);
                i.headingDeg = ic.number("heading_deg", i.headingDeg);
                i.pitchDeg = ic.number("pitch_deg", i.pitchDeg);
                i.rollDeg = ic.number("roll_deg", i.rollDeg);
                i.airspeedTrueMs = ic.number("airspeed_ms", i.airspeedTrueMs);
                i.onGround = ic.boolean("on_ground", i.onGround);
                const Json& cmd = v.child("command");
                if (!cmd.isNull()) {
                    if (!cmd.isObject()) throw Error(where + ": 'command' must be an object");
                    sv.command = parseCommand(cmd, where, sv.commandTarget);
                }
                parseEffects(v.child("effects"), sv.effects, where);
                sc.vehicles.push_back(std::move(sv));
            }
        }
    } catch (const std::exception& e) {
        throw Error(sc.source + ": " + e.what());
    }
    return sc;
}

Scenario loadScenario(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw Error("cannot open scenario " + path.string());
    std::stringstream ss;
    ss << in.rdbuf();
    Scenario sc = parseScenario(ss.str(), path.filename().string());
    // Relative paths in the file are relative to the file.
    const auto dir = path.parent_path();
    auto relativeTo = [&](std::string& p) {
        if (!p.empty() && std::filesystem::path(p).is_relative() && !dir.empty()) p = (dir / p).string();
    };
    relativeTo(sc.world.jsbsimRoot);
    relativeTo(sc.world.recordPath);
    return sc;
}

std::string dumpScenario(const Scenario& sc) {
    Json doc;
    {
        const WorldOptions d;
        const auto& o = sc.world;
        Json w;
        w.set("name", o.name).set("dt", o.dt).set("frame_skip", o.frameSkip);
        if (o.workers != d.workers) w.set("workers", static_cast<double>(o.workers));
        if (o.pinWorkers != d.pinWorkers) w.set("pin_workers", o.pinWorkers);
        if (o.seed != d.seed) w.set("seed", static_cast<double>(o.seed));
        if (o.capacity != d.capacity) w.set("capacity", static_cast<double>(o.capacity));
        if (o.publish != d.publish) w.set("publish", o.publish);
        if (o.publishIntervalSeconds != d.publishIntervalSeconds) w.set("publish_interval_s", o.publishIntervalSeconds);
        if (!o.jsbsimRoot.empty()) w.set("jsbsim_root", o.jsbsimRoot);
        if (o.terrain) w.set("terrain", true);
        if (!o.terrainUrl.empty()) w.set("terrain_url", o.terrainUrl);
        if (o.terrainZoom != d.terrainZoom) w.set("terrain_zoom", static_cast<double>(o.terrainZoom));
        if (!o.recordPath.empty()) w.set("record_path", o.recordPath);
        if (o.recordIntervalSeconds != d.recordIntervalSeconds) w.set("record_interval_s", o.recordIntervalSeconds);
        doc.set("world", std::move(w));
    }
    if (sc.hasEnvironment) {
        const auto& e = sc.environment;
        Json env;
        if (e.epochUtcSeconds > 0.0) env.set("unix_seconds", e.epochUtcSeconds);
        env.set("time_factor", e.timeFactor);
        env.set("wind", Json().set("direction_deg", e.windDirectionDeg).set("speed_ms", e.windSpeedMs).set("gust_ms", e.windGustMs).set("turbulence", e.turbulence));
        env.set("atmosphere", Json().set("temperature_sea_level_k", e.temperatureSeaLevelK).set("pressure_sea_level_pa", e.pressureSeaLevelPa).set("humidity", e.humidity));
        env.set("weather", Json().set("visibility_m", e.visibilityM).set("cloud_base_m", e.cloudBaseM).set("cloud_cover", e.cloudCover).set("precipitation", e.precipitation));
        doc.set("environment", std::move(env));
    }
    if (!sc.effects.empty()) doc.set("effects", dumpEffects(sc.effects));
    Json vehicles;
    for (const auto& sv : sc.vehicles) {
        Json v;
        v.set("name", sv.spec.name).set("type", sv.spec.type);
        if (!sv.spec.model.empty()) v.set("model", sv.spec.model);
        if (sv.spec.controlDivider != 1) v.set("control_divider", static_cast<double>(sv.spec.controlDivider));
        if (sv.count != 1) v.set("count", static_cast<double>(sv.count)).set("spacing_m", sv.spacingM);
        const auto& i = sv.spec.initial;
        Json ic;
        ic.set("lat_deg", i.latitudeDeg).set("lon_deg", i.longitudeDeg).set("alt_msl_m", i.altitudeMslM).set("heading_deg", i.headingDeg);
        if (i.pitchDeg != 0.0) ic.set("pitch_deg", i.pitchDeg);
        if (i.rollDeg != 0.0) ic.set("roll_deg", i.rollDeg);
        ic.set("airspeed_ms", i.airspeedTrueMs);
        if (i.onGround) ic.set("on_ground", true);
        v.set("initial", std::move(ic));
        if (sv.command) v.set("command", dumpCommand(*sv.command, sv.commandTarget));
        if (!sv.effects.empty()) v.set("effects", dumpEffects(sv.effects));
        vehicles.push(std::move(v));
    }
    doc.set("vehicles", vehicles.isNull() ? Json(Json::Array{}) : std::move(vehicles));
    return doc.dump();
}

namespace session {

std::vector<std::uint32_t> applyScenario(session::World& world, const Scenario& sc) {
    if (sc.hasEnvironment) {
        auto e = sc.environment;
        if (e.epochUtcSeconds <= 0.0) e.epochUtcSeconds = world.environment().epochUtcSeconds; // keep the clock
        else e.epochUtcSeconds -= world.simTime();
        e.revision = world.environment().revision;
        world.setEnvironment(e);
    }
    for (const auto& spec : sc.effects) {
        if (!effects::createBuiltinEffect(spec.id, spec.params)) throw Error(sc.source + ": unknown effect '" + spec.id + "'");
        world.addEffectToAll([spec] { return effects::createBuiltinEffect(spec.id, spec.params); });
    }

    struct Pending {
        std::uint32_t id;
        const ScenarioVehicle* vehicle;
    };
    std::vector<Pending> pending;
    std::vector<std::uint32_t> ids;
    for (const auto& sv : sc.vehicles) {
        for (unsigned k = 0; k < sv.count; ++k) {
            session::VehicleSpec s;
            s.name = sv.count > 1 ? sv.spec.name + "-" + std::to_string(k + 1) : sv.spec.name;
            s.type = sv.spec.type;
            s.model = sv.spec.model;
            s.controlDivider = sv.spec.controlDivider;
            s.initial = sv.spec.initial;
            if (k > 0) {
                // Abreast of the first instance, to the right of its heading.
                const double lat = s.initial.latitudeDeg * kDeg, lon = s.initial.longitudeDeg * kDeg;
                const double hdg = s.initial.headingDeg * kDeg;
                const double east = sv.spacingM * k * std::cos(hdg), north = -sv.spacingM * k * std::sin(hdg);
                double lat2, lon2;
                geo::offsetLatLon(lat, lon, north, east, lat2, lon2);
                s.initial.latitudeDeg = lat2 / kDeg;
                s.initial.longitudeDeg = lon2 / kDeg;
            }
            const std::uint32_t id = world.createVehicle(s);
            if (!id) throw Error(sc.source + ": cannot create vehicle '" + s.name + "' of type '" + s.type + "' (see log)");
            ids.push_back(id);
            pending.push_back({id, &sv});
            for (const auto& spec : sv.effects) {
                auto effect = effects::createBuiltinEffect(spec.id, spec.params);
                if (!effect) throw Error(sc.source + ": vehicle '" + s.name + "': unknown effect '" + spec.id + "'");
                world.addEffect(id, std::move(effect));
            }
        }
    }
    // Commands last: behaviour targets may be created later in the file.
    for (const auto& p : pending) {
        if (!p.vehicle->command) continue;
        control::Command command = *p.vehicle->command;
        if (auto* b = std::get_if<control::BehaviorCommand>(&command); b && !p.vehicle->commandTarget.empty()) {
            b->target = world.find(p.vehicle->commandTarget);
            if (!b->target) throw Error(sc.source + ": vehicle '" + p.vehicle->spec.name + "': unknown target '" + p.vehicle->commandTarget + "'");
        }
        if (!world.command(p.id, command)) throw Error(sc.source + ": vehicle '" + p.vehicle->spec.name + "': command rejected (see log)");
    }
    LOG_INFO("session") << "scenario " << sc.source << ": " << ids.size() << " vehicle(s)";
    return ids;
}

} // namespace session

} // namespace fsim
