// The performance tables (docs/flight-autonomy.md, SUB-02; ADR-29 FA-3a):
// read from an aircraft's profile, and looked up at a condition - linear in
// altitude and weight, and along each condition's level speeds at the same
// fraction of them. Apart from the runtime's code, last in the library, so
// that growing it moves nothing that flies.
#include "control/Profile.h"

#include <algorithm>
#include <charconv>
#include <cmath>

namespace fsim::control {

namespace {

/// A 2D table by name, and a 3D one.
std::vector<double>* table2(TablesSection& t, std::string_view name) noexcept {
    if (name == "min_tas_ms") return &t.minTasMs;
    if (name == "max_tas_ms") return &t.maxTasMs;
    if (name == "reach_tas_ms") return &t.reachTasMs;
    if (name == "stall_cas_ms") return &t.stallCasMs;
    if (name == "best_endurance_tas_ms") return &t.bestEnduranceTasMs;
    if (name == "best_endurance_fuel_kg_s") return &t.bestEnduranceFuelKgS;
    if (name == "best_range_tas_ms") return &t.bestRangeTasMs;
    if (name == "best_range_fuel_kg_s") return &t.bestRangeFuelKgS;
    if (name == "best_endurance_power_w") return &t.bestEndurancePowerW;
    if (name == "best_range_power_w") return &t.bestRangePowerW;
    if (name == "max_climb_ms") return &t.maxClimbMs;
    if (name == "climb_tas_ms") return &t.climbTasMs;
    return nullptr;
}

std::vector<double>* table3(TablesSection& t, std::string_view name) noexcept {
    if (name == "fuel_kg_s") return &t.fuelKgS;
    if (name == "power_w") return &t.powerW;
    if (name == "ps_full_ms") return &t.psFullMs;
    if (name == "ps_idle_ms") return &t.psIdleMs;
    return nullptr;
}

std::vector<double>* axis(TablesSection& t, std::string_view name) noexcept {
    if (name == "altitude_m") return &t.altitudeM;
    if (name == "weight_kg") return &t.weightKg;
    if (name == "speed_fraction") return &t.speedFraction;
    return nullptr;
}

/// A capacity by name: the fuel's, a battery's.
double* capacity(TablesSection& t, std::string_view name) noexcept {
    if (name == "fuel_capacity_kg") return &t.fuelCapacityKg;
    if (name == "battery_capacity_j") return &t.batteryCapacityJ;
    return nullptr;
}

/// "h3" -> 3 for tag 'h'; -1 if it is not one.
int index(std::string_view part, char tag) noexcept {
    if (part.size() < 2 || part[0] != tag) return -1;
    int i = -1;
    const auto r = std::from_chars(part.data() + 1, part.data() + part.size(), i);
    return r.ec == std::errc() && r.ptr == part.data() + part.size() ? i : -1;
}

/// A path's parts: "max_tas_ms/h0/w2" -> {"max_tas_ms", "h0", "w2"}.
std::vector<std::string_view> parts(std::string_view path) {
    std::vector<std::string_view> out;
    while (!path.empty()) {
        const auto slash = path.find('/');
        out.push_back(path.substr(0, slash));
        if (slash == std::string_view::npos) break;
        path.remove_prefix(slash + 1);
    }
    return out;
}

/// Where x lies between a rising axis's two points: the lower's index and the fraction to the next; false if x
/// is outside [a.front() - before, a.back() + after]. One point: itself.
bool bracket(const std::vector<double>& a, double x, double before, double after, std::size_t& i, double& f) noexcept {
    if (a.empty() || !std::isfinite(x) || x < a.front() - before || x > a.back() + after) return false;
    if (a.size() == 1) {
        i = 0, f = 0.0;
        return true;
    }
    i = static_cast<std::size_t>(std::upper_bound(a.begin(), a.end(), x) - a.begin());
    i = std::clamp<std::size_t>(i, 1, a.size() - 1) - 1;
    f = (x - a[i]) / (a[i + 1] - a[i]);
    return true;
}

/// Linear from a to b at f (beyond them too); on either point (within a billionth) its own value, whatever the
/// other's: a condition or level point beside one not flown is still read where it was.
double mix(double a, double b, double f) noexcept {
    if (std::abs(f) < 1e-9) return a;
    if (std::abs(f - 1.0) < 1e-9) return b;
    return a + f * (b - a);
}

/// The four conditions a lookup lies between, and their weights: altitude clamped below the lowest, NaN above
/// the highest; weight on down to the tanks empty (the zero-fuel weight: the heaviest less the capacity).
struct Corners {
    std::size_t h = 0, w = 0;
    double fh = 0.0, fw = 0.0;
};

bool corners(const TablesSection& t, double altitudeM, double weightKg, Corners& c) noexcept {
    const double below = t.altitudeM.empty() ? 0.0 : t.altitudeM.front(); // (clamped: the lowest is a little above the ground)
    if (!bracket(t.altitudeM, std::max(altitudeM, t.altitudeM.empty() ? altitudeM : t.altitudeM.front()), below, 1.0, c.h, c.fh)) return false;
    const double empty = std::isfinite(t.fuelCapacityKg) && !t.weightKg.empty() ? t.weightKg.back() - t.fuelCapacityKg : t.weightKg.front();
    const double lightest = std::min(t.weightKg.front(), empty);
    if (!bracket(t.weightKg, weightKg, t.weightKg.front() - lightest, 1.0, c.w, c.fw)) return false;
    c.fh = std::clamp(c.fh, 0.0, 1.0); // (in altitude: never beyond the rows flown)
    return true;
}

/// A 2D table at the corners: bilinear (linear in weight beyond the lightest flown); NaN if a corner it needs is.
double blend2(const TablesSection& t, const std::vector<double>& v, const Corners& c) noexcept {
    const std::size_t nw = t.weightKg.size();
    if (v.size() != t.altitudeM.size() * nw) return kUnknown;
    const std::size_t h1 = std::min(c.h + 1, t.altitudeM.size() - 1), w1 = std::min(c.w + 1, nw - 1);
    auto at = [&](std::size_t h, std::size_t w) { return v[h * nw + w]; };
    auto row = [&](std::size_t h) {
        const double a = at(h, c.w), b = at(h, w1);
        return mix(a, b, c.fw);
    };
    return mix(row(c.h), row(h1), c.fh);
}

/// A condition's 3D table at a fraction of its level speeds: linear between its points (on a little beyond its
/// last two, to its top speed); NaN outside.
double along(const TablesSection& t, const std::vector<double>& v, std::size_t cond, double x) noexcept {
    const std::size_t nk = t.speedFraction.size();
    if (nk < 2 || v.size() < (cond + 1) * nk) return kUnknown;
    const double* p = v.data() + cond * nk;
    std::size_t k = 0;
    double f = 0.0;
    if (!bracket(t.speedFraction, x, 0.02, 0.06, k, f)) return kUnknown;
    return mix(p[k], p[k + 1], f);
}

} // namespace

void readTables(const std::vector<std::pair<std::string, double>>& values, TablesSection& out, const std::string& where,
                std::vector<std::string>& warnings) {
    // the axes first: their sizes set the tables'
    for (const auto& [path, value] : values) {
        const auto p = parts(path);
        if (p.size() != 2) continue;
        for (const auto& [name, tag] : {std::pair<const char*, char>{"altitude_m", 'h'}, {"weight_kg", 'w'}, {"speed_fraction", 'v'}})
            if (p[0] == name) {
                const int i = index(p[1], tag);
                std::vector<double>& a = *axis(out, name);
                if (i < 0 || i > 255) continue;
                if (a.size() <= static_cast<std::size_t>(i)) a.resize(static_cast<std::size_t>(i) + 1, kUnknown);
                a[static_cast<std::size_t>(i)] = value;
            }
    }
    const std::size_t nh = out.altitudeM.size(), nw = out.weightKg.size(), nk = out.speedFraction.size();
    for (auto* a : {&out.altitudeM, &out.weightKg, &out.speedFraction})
        if (std::adjacent_find(a->begin(), a->end(), [](double x, double y) { return !(x < y); }) != a->end() ||
            std::any_of(a->begin(), a->end(), [](double x) { return !std::isfinite(x); })) {
            warnings.push_back(where + ": an axis is not finite and strictly rising; the tables are ignored");
            const SectionHeader header = out.header;
            out = TablesSection{};
            out.header = header;
            return;
        }
    for (const char* name : {"min_tas_ms", "max_tas_ms", "reach_tas_ms", "stall_cas_ms", "best_endurance_tas_ms", "best_endurance_fuel_kg_s",
                             "best_range_tas_ms", "best_range_fuel_kg_s", "best_endurance_power_w", "best_range_power_w", "max_climb_ms",
                             "climb_tas_ms"})
        table2(out, name)->assign(nh * nw, kUnknown);
    for (const char* name : {"fuel_kg_s", "power_w", "ps_full_ms", "ps_idle_ms"}) table3(out, name)->assign(nh * nw * nk, kUnknown);
    for (const auto& [path, value] : values) {
        const auto p = parts(path);
        if (p.empty() || p[0] == "version" || p[0] == "provenance" || axis(out, p[0])) continue;
        if (double* c = p.size() == 1 ? capacity(out, p[0]) : nullptr) {
            *c = value;
            continue;
        }
        const int h = p.size() >= 3 ? index(p[1], 'h') : -1, w = p.size() >= 3 ? index(p[2], 'w') : -1;
        const bool inside = h >= 0 && w >= 0 && static_cast<std::size_t>(h) < nh && static_cast<std::size_t>(w) < nw;
        const std::size_t cond = inside ? static_cast<std::size_t>(h) * nw + static_cast<std::size_t>(w) : 0;
        if (std::vector<double>* v = p.size() == 3 ? table2(out, p[0]) : nullptr; v && inside) {
            (*v)[cond] = value;
            continue;
        }
        if (std::vector<double>* v = p.size() == 4 ? table3(out, p[0]) : nullptr; v && inside) {
            const int k = index(p[3], 'v');
            if (k >= 0 && static_cast<std::size_t>(k) < nk) {
                (*v)[cond * nk + static_cast<std::size_t>(k)] = value;
                continue;
            }
        }
        warnings.push_back(where + "/" + path + " is not a table cell this build knows; ignored");
    }
}

double tableValue(const TablesSection& tables, std::string_view path) noexcept {
    auto& t = const_cast<TablesSection&>(tables);
    const auto p = parts(path);
    if (const double* c = p.size() == 1 ? capacity(t, p[0]) : nullptr) return *c;
    if (p.size() == 2)
        for (const auto& [name, tag] : {std::pair<const char*, char>{"altitude_m", 'h'}, {"weight_kg", 'w'}, {"speed_fraction", 'v'}})
            if (p[0] == name) {
                const int i = index(p[1], tag);
                const std::vector<double>& a = *axis(t, name);
                return i >= 0 && static_cast<std::size_t>(i) < a.size() ? a[static_cast<std::size_t>(i)] : kUnknown;
            }
    if (p.size() < 3) return kUnknown;
    const int h = index(p[1], 'h'), w = index(p[2], 'w');
    const std::size_t nh = t.altitudeM.size(), nw = t.weightKg.size(), nk = t.speedFraction.size();
    if (h < 0 || w < 0 || static_cast<std::size_t>(h) >= nh || static_cast<std::size_t>(w) >= nw) return kUnknown;
    const std::size_t cond = static_cast<std::size_t>(h) * nw + static_cast<std::size_t>(w);
    if (const std::vector<double>* v = p.size() == 3 ? table2(t, p[0]) : nullptr) return cond < v->size() ? (*v)[cond] : kUnknown;
    if (const std::vector<double>* v = p.size() == 4 ? table3(t, p[0]) : nullptr) {
        const int k = index(p[3], 'v');
        return k >= 0 && static_cast<std::size_t>(k) < nk && cond * nk + static_cast<std::size_t>(k) < v->size() ? (*v)[cond * nk + static_cast<std::size_t>(k)]
                                                                                                                 : kUnknown;
    }
    return kUnknown;
}

TablesAt tablesAt(const TablesSection& t, double altitudeM, double weightKg) noexcept {
    TablesAt out;
    Corners c;
    if (t.empty() || !corners(t, altitudeM, weightKg, c)) return out;
    out.minTasMs = blend2(t, t.minTasMs, c), out.maxTasMs = blend2(t, t.maxTasMs, c), out.reachTasMs = blend2(t, t.reachTasMs, c);
    out.stallCasMs = blend2(t, t.stallCasMs, c);
    out.bestEnduranceTasMs = blend2(t, t.bestEnduranceTasMs, c), out.bestEnduranceFuelKgS = blend2(t, t.bestEnduranceFuelKgS, c);
    out.bestRangeTasMs = blend2(t, t.bestRangeTasMs, c), out.bestRangeFuelKgS = blend2(t, t.bestRangeFuelKgS, c);
    out.bestEndurancePowerW = blend2(t, t.bestEndurancePowerW, c), out.bestRangePowerW = blend2(t, t.bestRangePowerW, c);
    out.maxClimbMs = blend2(t, t.maxClimbMs, c), out.climbTasMs = blend2(t, t.climbTasMs, c);
    return out;
}

TablesAtSpeed tablesAt(const TablesSection& t, double altitudeM, double weightKg, double tasMs) noexcept {
    TablesAtSpeed out;
    Corners c;
    if (t.empty() || !corners(t, altitudeM, weightKg, c) || !std::isfinite(tasMs)) return out;
    // the fraction of its level speeds it lies at, from the bounds between the conditions...
    const double lo = blend2(t, t.minTasMs, c), hi = 0.97 * blend2(t, t.maxTasMs, c);
    if (!(hi > lo)) return out;
    const double x = (tasMs - lo) / (hi - lo);
    // ...and each condition's table at that fraction of its own, blended as a 2D table's cells are
    const std::size_t nw = t.weightKg.size(), h1 = std::min(c.h + 1, t.altitudeM.size() - 1), w1 = std::min(c.w + 1, nw - 1);
    auto blend = [&](const std::vector<double>& v) {
        auto row = [&](std::size_t h) {
            const double a = along(t, v, h * nw + c.w, x), b = along(t, v, h * nw + w1, x);
            return mix(a, b, c.fw);
        };
        return mix(row(c.h), row(h1), c.fh);
    };
    out.fuelKgS = blend(t.fuelKgS), out.powerW = blend(t.powerW), out.psFullMs = blend(t.psFullMs), out.psIdleMs = blend(t.psIdleMs);
    return out;
}

double tablesCeilingM(const TablesSection& t, double weightKg) noexcept {
    if (t.empty() || t.altitudeM.size() < 2) return kUnknown;
    // each weight's ceiling, then linear in weight
    const std::size_t nh = t.altitudeM.size(), nw = t.weightKg.size();
    auto ceilingAt = [&](std::size_t w) {
        constexpr double kServiceMs = 0.508; // 100 ft/min
        auto climb = [&](std::size_t h) { return h * nw + w < t.maxClimbMs.size() ? t.maxClimbMs[h * nw + w] : kUnknown; };
        std::size_t last = nh; // the highest altitude so far that climbs
        for (std::size_t h = 0; h < nh; ++h) {
            const double c = climb(h);
            if (std::isfinite(c) && c >= kServiceMs) {
                last = h;
                continue;
            }
            if (last == nh) continue;
            const double below = climb(last), here = std::isfinite(c) ? c : 0.0; // (nothing held level there: no climb)
            return t.altitudeM[last] + (below - kServiceMs) / (below - here) * (t.altitudeM[h] - t.altitudeM[last]);
        }
        if (last == nh - 1 && last >= 1 && std::isfinite(climb(last - 1)) && climb(last - 1) > climb(last)) {
            // (no further than as high again: a fixed wing's rows run to near its ceiling, a light one's reached at
            // most 42 % above them; a rotorcraft still climbing hard at 3,000 m has its ceiling tens of times higher,
            // where nothing was flown)
            const double above = (climb(last) - kServiceMs) / (climb(last - 1) - climb(last)) * (t.altitudeM[last] - t.altitudeM[last - 1]);
            return above <= t.altitudeM[last] ? t.altitudeM[last] + above : kUnknown;
        }
        return kUnknown;
    };
    Corners c;
    if (!corners(t, t.altitudeM.front(), weightKg, c)) return kUnknown;
    return mix(ceilingAt(c.w), ceilingAt(std::min(c.w + 1, nw - 1)), c.fw);
}

} // namespace fsim::control
