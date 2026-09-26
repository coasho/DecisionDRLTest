// The control architecture's compatibility promise (docs/control-architecture.md,
// sections 12.4 and 13): a stock aircraft commanded through the existing
// entry points flies as it did before the architecture changed.
#include "control_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace fsim;

TEST_CASE("the stock c172x flies its closed-loop flights as recorded before the control architecture changed", "[control][world]") {
    // name, world step -> the checkpoint's values (tests/data, written by fsim_control_bench checkpoints)
    std::map<std::pair<std::string, int>, std::vector<double>> recorded;
    std::ifstream in(FSIM_TEST_DATA_DIR "/c172x_checkpoints.txt");
    REQUIRE(in);
    for (std::string line; std::getline(in, line);) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream fields(line);
        std::string name;
        int step = 0;
        fields >> name >> step;
        std::vector<double> values;
        for (double v; fields >> v;) values.push_back(v);
        recorded[{name, step}] = values;
    }
    REQUIRE(recorded.size() == 6 * static_cast<std::size_t>(flights::kCheckpointSteps / flights::kCheckpointEvery));

    session::World w(flights::worldOptions("test-control-compat", FSIM_TEST_JSBSIM_ROOT));
    flights::Flights f(w, false);
    std::size_t compared = 0;
    for (int k = 0; k < flights::kCheckpointSteps; ++k) {
        f.drive(k);
        w.step();
        if ((k + 1) % flights::kCheckpointEvery) continue;
        for (const auto& fl : f.flights()) {
            if (!fl.closedLoop) continue;
            const auto it = recorded.find({fl.name, k + 1});
            REQUIRE(it != recorded.end());
            const auto now = flights::checkpoint(*w.vehicleState(fl.id), *w.inputs(fl.id));
            REQUIRE(now.size() == it->second.size());
            for (std::size_t i = 0; i < now.size(); ++i) {
                // relative for large values, absolute below 1: a real change is far larger
                const double tolerance = 1e-9 * std::max(1.0, std::abs(it->second[i]));
                INFO(fl.name << " at step " << k + 1 << ", value " << i << ": " << now[i] << " recorded " << it->second[i]);
                CHECK(std::abs(now[i] - it->second[i]) <= tolerance);
            }
            ++compared;
        }
    }
    REQUIRE(compared == recorded.size());
}
