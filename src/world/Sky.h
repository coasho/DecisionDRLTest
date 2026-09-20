#pragma once

#include <vsg/all.h>

namespace fsim::world {

/// Sun direction in ECEF for a UTC time of year/day (design 8.2 "Sky / lighting").
/// Approximate solar position (declination from day of year, hour angle from
/// UTC, no equation of time): good to ~2 degrees, which is all lighting needs.
vsg::dvec3 sunDirectionEcef(int dayOfYear, double utcHours);

/// Directional sun light plus a little ambient, positioned by sunDirectionEcef().
/// Replaces the headlight for scenes with terrain relief.
vsg::ref_ptr<vsg::Node> createSunLight(int dayOfYear, double utcHours, float sunIntensity = 1.0f, float ambient = 0.18f);

/// Day of year (1-366) and UTC hour (0-24) of "now".
void currentUtc(int& dayOfYear, double& utcHours);

/// Day of year and UTC hour of a Unix time (seconds).
void utcOf(double unixSeconds, int& dayOfYear, double& utcHours);

/// Re-aim the sun of a group made by createSunLight().
void setSunDirection(vsg::Node* sunLight, const vsg::dvec3& toSunEcef);

} // namespace fsim::world
