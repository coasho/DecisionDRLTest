#include "core/Units.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using Catch::Matchers::WithinRel;
using namespace fsim::units;

TEST_CASE("length conversions round-trip", "[units]") {
    CHECK_THAT(feetToMetres(1.0), WithinRel(0.3048, 1e-12));
    CHECK_THAT(metresToFeet(feetToMetres(12345.678)), WithinRel(12345.678, 1e-12));
}

TEST_CASE("speed conversions", "[units]") {
    CHECK_THAT(knotsToMetresPerSecond(100.0), WithinRel(51.4444444, 1e-7));
    CHECK_THAT(metresPerSecondToKnots(knotsToMetresPerSecond(250.0)), WithinRel(250.0, 1e-12));
}

TEST_CASE("angle conversions", "[units]") {
    CHECK_THAT(degreesToRadians(180.0), WithinRel(kPi, 1e-15));
    CHECK_THAT(radiansToDegrees(kPi / 2.0), WithinRel(90.0, 1e-15));
}

TEST_CASE("force conversion", "[units]") {
    CHECK_THAT(poundsForceToNewtons(1.0), WithinRel(4.4482216152605, 1e-12));
}
