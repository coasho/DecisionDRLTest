#include "core/Rng.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <set>

using fsim::Rng;

TEST_CASE("same seed gives same sequence", "[rng]") {
    Rng a(42), b(42);
    for (int i = 0; i < 1000; ++i) REQUIRE(a.next() == b.next());
}

TEST_CASE("vehicle streams are distinct and reproducible", "[rng]") {
    std::set<std::uint64_t> firstValues;
    for (std::uint64_t env = 0; env < 8; ++env)
        for (std::uint64_t veh = 0; veh < 8; ++veh) {
            Rng r1 = Rng::forVehicle(7, env, veh);
            Rng r2 = Rng::forVehicle(7, env, veh);
            const auto v = r1.next();
            REQUIRE(v == r2.next());
            firstValues.insert(v);
        }
    CHECK(firstValues.size() == 64); // no collisions across (env, vehicle)
}

TEST_CASE("uniform stays in range and normal is roughly standard", "[rng]") {
    Rng r(3);
    double sum = 0.0, sumSq = 0.0;
    constexpr int n = 200000;
    for (int i = 0; i < n; ++i) {
        const double u = r.uniform();
        REQUIRE(u >= 0.0);
        REQUIRE(u < 1.0);
        const double z = r.normal();
        sum += z;
        sumSq += z * z;
    }
    const double mean = sum / n;
    const double var = sumSq / n - mean * mean;
    CHECK(std::abs(mean) < 0.02);
    CHECK(std::abs(var - 1.0) < 0.03);
}
