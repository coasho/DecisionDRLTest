// Operational lines' geometry (docs/flight-autonomy.md, 4.44; A-GRA's LineType): see Lines.h.
#include "control/Lines.h"

#include "core/Geodesy.h"

#include <algorithm>
#include <cmath>

namespace fsim::control::lines {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool code(double v, double count) noexcept { return isHold(v) || (v == std::floor(v) && v >= 0.0 && v < count); }
bool finiteOr(double v) noexcept { return isHold(v) || std::isfinite(v); }
bool width(double v) noexcept { return isHold(v) || (std::isfinite(v) && v >= 0.0); }
AltitudeReference reference(double v) noexcept { return isHold(v) ? AltitudeReference::Msl : static_cast<AltitudeReference>(static_cast<int>(v)); }

/// A band: finite, in order, its reference a code.
bool band(double lo, double hi, double ref) noexcept {
    if (!finiteOr(lo) || !finiteOr(hi) || !code(ref, static_cast<double>(AltitudeReference::Count))) return false;
    return isHold(lo) || isHold(hi) || lo <= hi;
}

} // namespace

int fault(const OpLine& l, bool frameKnown) noexcept {
    const bool framed = !isHold(l.frame);
    if (framed && !(l.frame == std::floor(l.frame) && l.frame >= 1.0 && frameKnown)) return 4;
    if (!code(l.frameRotation, static_cast<double>(FrameRotation::Count)) || (!framed && !isHold(l.frameRotation))) return 4;
    // its velocity: both ways or neither, finite; a frame moves it instead
    if (isHold(l.northMs) != isHold(l.eastMs) || !finiteOr(l.northMs) || !finiteOr(l.eastMs) || !finiteOr(l.timeS)) return 5;
    if ((framed && !isHold(l.northMs)) || (isHold(l.northMs) && !isHold(l.timeS))) return 5;
    if (!band(l.altitudeMinM, l.altitudeMaxM, l.altitudeReference)) return 3;
    if (!isHold(l.altitudeReference) && isHold(l.altitudeMinM) && isHold(l.altitudeMaxM)) return 3;
    if (!width(l.leftWidthM) || !width(l.rightWidthM)) return 2;
    if (!code(l.projection, static_cast<double>(Projection::Count))) return 1;
    if (l.vertices.size() < 2 || l.vertices.size() > MustFlyArea::kVertices) return 0;
    for (const LineVertex& v : l.vertices) {
        const bool placed = framed ? isHold(v.latitudeRad) && isHold(v.longitudeRad) && std::isfinite(v.xM) && std::isfinite(v.yM)
                                   : isHold(v.xM) && isHold(v.yM) && std::isfinite(v.latitudeRad) && std::isfinite(v.longitudeRad) &&
                                         std::abs(v.latitudeRad) <= 0.5 * kPi;
        if (!placed || !finiteOr(v.altitudeM) || !band(v.altitudeMinM, v.altitudeMaxM, v.altitudeReference)) return 0;
        const bool own = !isHold(v.altitudeMinM) || !isHold(v.altitudeMaxM);
        if (!isHold(v.altitudeReference) && isHold(v.altitudeM) && !own) return 0; // (a reference to read nothing in)
        if (!width(v.leftWidthM) || !width(v.rightWidthM)) return 0;
        // its altitude within the band there - its own range, else the line's - where both are in one reference
        const double lo = own ? v.altitudeMinM : l.altitudeMinM, hi = own ? v.altitudeMaxM : l.altitudeMaxM;
        const bool same = own || reference(v.altitudeReference) == reference(l.altitudeReference);
        if (!isHold(v.altitudeM) && same && ((!isHold(lo) && v.altitudeM < lo) || (!isHold(hi) && v.altitudeM > hi))) return 0;
    }
    // two in a row at one place (within a metre, laid out as flown): no segment between them to fly
    MustFlyArea a;
    FrameSpec none;
    layOut(l, framed ? &none : nullptr, 0.0, a);
    for (std::size_t i = 0; i + 1 < a.lineCount; ++i)
        if (std::hypot(a.vertices[i + 1][0] - a.vertices[i][0], a.vertices[i + 1][1] - a.vertices[i][1]) < 1.0) return 0;
    return -1;
}

void layOut(const OpLine& l, const FrameSpec* spec, double nowS, MustFlyArea& a) noexcept {
    a = MustFlyArea{};
    a.framed = spec != nullptr;
    if (spec)
        a.frame = *spec, a.frameId = static_cast<FrameId>(l.frame),
        a.rotation = isHold(l.frameRotation) ? FrameRotation::Unrotated : static_cast<FrameRotation>(static_cast<int>(l.frameRotation));
    a.northMs = orHold(l.northMs, 0.0), a.eastMs = orHold(l.eastMs, 0.0), a.timeS = orHold(l.timeS, nowS);
    a.projection = isHold(l.projection) ? Projection::GreatCircle : static_cast<Projection>(static_cast<int>(l.projection));
    const std::size_t n = std::min(l.vertices.size(), MustFlyArea::kVertices);
    // its reference: a framed line's the frame's origin; else its vertices' mean
    double lat0 = 0.0, lon0 = 0.0;
    if (!spec && n) {
        const double first = l.vertices[0].longitudeRad;
        for (std::size_t i = 0; i < n; ++i) lat0 += l.vertices[i].latitudeRad, lon0 += first + geo::wrapPi(l.vertices[i].longitudeRad - first);
        lat0 /= static_cast<double>(n), lon0 /= static_cast<double>(n);
        a.latitudeRad = lat0, a.longitudeRad = geo::wrapPi(lon0);
    }
    for (std::size_t i = 0; i < n; ++i) {
        const LineVertex& v = l.vertices[i];
        if (spec) a.vertices[i][0] = v.xM, a.vertices[i][1] = v.yM;
        else geo::localNorthEastM(lat0, lon0, v.latitudeRad, lon0 + geo::wrapPi(v.longitudeRad - lon0), a.vertices[i][0], a.vertices[i][1]);
        const bool own = !isHold(v.altitudeMinM) || !isHold(v.altitudeMaxM);
        a.lineAltitudeM[i] = v.altitudeM, a.lineAltitudeReference[i] = reference(v.altitudeReference);
        a.lineMinM[i] = own ? v.altitudeMinM : l.altitudeMinM, a.lineMaxM[i] = own ? v.altitudeMaxM : l.altitudeMaxM;
        a.lineBandReference[i] = reference(own ? v.altitudeReference : l.altitudeReference);
        a.leftWidthM[i] = isHold(v.leftWidthM) ? l.leftWidthM : v.leftWidthM;
        a.rightWidthM[i] = isHold(v.rightWidthM) ? l.rightWidthM : v.rightWidthM;
    }
    a.lineCount = static_cast<std::uint8_t>(n);
}

} // namespace fsim::control::lines
