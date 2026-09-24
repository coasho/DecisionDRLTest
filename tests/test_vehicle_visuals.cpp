// Moving control surfaces (VSG scene graph, no Vulkan): the nodes a model
// names fsim:<channel> turn with each vehicle's own deflections, the way
// JSBSim means them, and the geometry under them stays shared.
#include "world/VehicleVisuals.h"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>

#include <vsgXchange/all.h>

#include <filesystem>
#include <optional>
#include <vector>

using namespace fsim;
using Joint = world::VehicleVisuals::Joint;

namespace {

/// The matrix from a graph's root down to one transform in it (inclusive).
class MatrixTo : public vsg::Visitor {
public:
    explicit MatrixTo(const vsg::Node* target) : target_(target) {}
    std::optional<vsg::dmat4> found;

    void apply(vsg::Object& o) override {
        if (!found) o.traverse(*this);
    }
    void apply(vsg::Transform& t) override {
        if (found) return;
        const vsg::dmat4 outer = m_;
        m_ = t.transform(m_);
        if (&t == target_) found = m_;
        else t.traverse(*this);
        m_ = outer;
    }

private:
    const vsg::Node* target_;
    vsg::dmat4 m_;
};

vsg::dmat4 matrixTo(const vsg::ref_ptr<vsg::Node>& root, const vsg::Node* target) {
    MatrixTo v(target);
    root->accept(v);
    REQUIRE(v.found);
    return *v.found;
}

vsg::dvec3 point(const vsg::dmat4& m, const vsg::dvec3& p) {
    const vsg::dvec4 r = m * vsg::dvec4(p.x, p.y, p.z, 1.0);
    return vsg::dvec3(r.x, r.y, r.z);
}

} // namespace

TEST_CASE("control surfaces turn with each vehicle's own deflections", "[render][visuals]") {
    const std::filesystem::path dir = FSIM_TEST_AIRCRAFT_DIR;
    REQUIRE(std::filesystem::exists(dir / "c172" / "c172.glb")); // written by tools/hangar

    world::VehicleVisuals::Settings settings; // no default model: the placeholder
    settings.modelDirs.push_back(dir);
    world::VehicleVisuals visuals(2, settings, vsg::Options::create(vsgXchange::all::create()));
    visuals.setModel(0, {}, "jsbsim:c172");
    visuals.setModel(1, {}, "jsbsim:c172");
    const auto& a = visuals.pose(0);
    const auto& b = visuals.pose(1);

    // flaps, ailerons and elevator both sides, one rudder - and the propeller
    const auto count = [](const auto& joints, Joint::Kind kind) {
        return std::count_if(joints.begin(), joints.end(), [kind](const Joint& j) { return j.kind == kind; });
    };
    REQUIRE(count(a.joints, Joint::Surface) == 7);
    REQUIRE(count(a.joints, Joint::Propeller) == 1);
    REQUIRE(b.joints.size() == a.joints.size());
    REQUIRE(a.normal);
    REQUIRE(a.normal != b.normal);
    for (std::size_t j = 0; j < a.transforms.size(); ++j) {
        REQUIRE(a.transforms[j] != b.transforms[j]);            // each vehicle turns its own
        REQUIRE(a.transforms[j]->children == b.transforms[j]->children); // and draws the same geometry
    }

    // At rest, then vehicle 0 with every channel positive; vehicle 1 stays at rest.
    std::vector<sim::VehicleState> states(2);
    visuals.update(Span<const sim::VehicleState>(states));
    std::vector<vsg::dmat4> rest;
    for (const auto& t : a.transforms) rest.push_back(matrixTo(a.normal, t.get()));
    states[0].aileronRad = states[0].elevatorRad = states[0].rudderRad = states[0].flapsRad = 0.25;
    visuals.update(Span<const sim::VehicleState>(states));

    int left = 0, right = 0;
    for (std::size_t j = 0; j < a.transforms.size(); ++j) {
        const vsg::dmat4 before = rest[j];
        const vsg::dmat4 after = matrixTo(a.normal, a.transforms[j].get());
        if (a.joints[j].kind != Joint::Surface) { // the propeller turns with the clock, not the controls
            REQUIRE(after == before);
            continue;
        }
        // A point on the surface behind the hinge (body axes: x forward, y right,
        // z down): the local direction that points aft at rest, off the hinge axis.
        const vsg::dmat4 inv = vsg::inverse(before);
        vsg::dvec3 aft = point(inv, point(before, vsg::dvec3()) + vsg::dvec3(-1.0, 0.0, 0.0));
        aft.x = 0.0; // local x is the hinge
        const vsg::dvec3 probe = vsg::normalize(aft) * 0.3;
        const vsg::dvec3 moved = point(after, probe) - point(before, probe);
        const bool onLeft = point(before, vsg::dvec3()).y < 0.0;
        CAPTURE(j, a.joints[j].channel, onLeft, moved.x, moved.y, moved.z);
        switch (a.joints[j].channel) {
        case Joint::Elevator:
        case Joint::Flaps: REQUIRE(moved.z > 0.05); break;                  // trailing edge down
        case Joint::Aileron:                                                 // left down, right up
            REQUIRE((onLeft ? moved.z : -moved.z) > 0.05);
            ++(onLeft ? left : right);
            break;
        case Joint::Rudder: REQUIRE(moved.y < -0.05); break;                 // trailing edge left
        }
        // vehicle 1 did not move
        REQUIRE(matrixTo(b.normal, b.transforms[j].get()) == before);
    }
    REQUIRE(left == 1);
    REQUIRE(right == 1);
}

TEST_CASE("control surface names parse, and anything else is left alone", "[render][visuals]") {
    Joint j;
    REQUIRE(Joint::parse("fsim:elevator", j));
    REQUIRE(j.channel == Joint::Elevator);
    REQUIRE(j.gain == 1.0);
    REQUIRE(Joint::parse("fsim:aileron:-0.5", j));
    REQUIRE(j.channel == Joint::Aileron);
    REQUIRE(j.gain == -0.5);
    REQUIRE(Joint::parse("fsim:flap", j));
    REQUIRE(j.channel == Joint::Flaps);
    REQUIRE(j.mix.empty());
    // a stabilator that rolls too, a flaperon
    REQUIRE(Joint::parse("fsim:elevator+aileron:-0.25", j));
    REQUIRE(j.channel == Joint::Elevator);
    REQUIRE(j.gain == 1.0);
    REQUIRE(j.mix.size() == 1);
    REQUIRE(j.mix[0].first == Joint::Aileron);
    REQUIRE(j.mix[0].second == -0.25);
    REQUIRE(Joint::parse("fsim:aileron+flaps:-1", j));
    REQUIRE(j.mix.size() == 1);
    sim::VehicleState s;
    s.aileronRad = 0.1;
    s.flapsRad = 0.3;
    j.rest = vsg::dmat4();
    const vsg::dmat4 m = j.matrix(s); // turned by 0.1 - 0.3 about x
    REQUIRE(std::abs(std::atan2(m[1][2], m[1][1]) - (-0.2)) < 1e-12);
    // stops: a canard that travels further than the elevons on its channel,
    // an elevon held at its own when pitch and roll add up
    REQUIRE(Joint::parse("fsim:elevator:-1@-50,20", j));
    REQUIRE(j.gain == -1.0);
    s = sim::VehicleState();
    s.elevatorRad = 0.8; // 46 deg nose down: the canard turns -46, within its -50
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1]) - (-0.8)) < 1e-12);
    REQUIRE(Joint::parse("fsim:elevator+aileron:-1@-25,25", j));
    s.elevatorRad = 0.3;
    s.aileronRad = -0.3; // 0.6 rad asked, held at 25 deg
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1]) - 25.0 * 3.14159265358979323846 / 180.0) < 1e-12);
    REQUIRE(Joint::parse("fsim:elevator", j)); // no stops: turns as far as asked
    s.elevatorRad = 0.9;
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1]) - 0.9) < 1e-12);
    REQUIRE_FALSE(Joint::parse("fsim:canopy", j));
    REQUIRE_FALSE(Joint::parse("fsim:rudder:x", j));
    REQUIRE_FALSE(Joint::parse("fsim:elevator+", j));
    REQUIRE_FALSE(Joint::parse("fsim:elevator+canopy", j));
    // landing gear: a leg that turns 95 deg as the gear goes up (1 -> 0), a
    // door that opens in the first fifth of the way down
    REQUIRE(Joint::parse("fsim:gear:95", j));
    REQUIRE(j.kind == Joint::Gear);
    s = sim::VehicleState();
    s.gearPosition = 1.0;
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1])) < 1e-12);
    s.gearPosition = 0.5;
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1]) - 47.5 * 3.14159265358979323846 / 180.0) < 1e-9);
    REQUIRE(Joint::parse("fsim:gear:-80:0:0.2", j));
    s.gearPosition = 0.1;
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1]) + 40.0 * 3.14159265358979323846 / 180.0) < 1e-9);
    s.gearPosition = 1.0;
    REQUIRE(std::abs(std::atan2(j.matrix(s)[1][2], j.matrix(s)[1][1]) + 80.0 * 3.14159265358979323846 / 180.0) < 1e-9);
    // everything else moves with what the simulation reports, not with a
    // formula of the viewer's: an afterburner plume by the engine's afterburner
    // (hidden when it is out), full length when it is full
    const auto turn = [](const vsg::dmat4& mat) { return std::atan2(mat[1][2], mat[1][1]); };
    constexpr double kRad = 3.14159265358979323846 / 180.0;
    REQUIRE(Joint::parse("fsim:afterburner:1", j));
    REQUIRE(j.kind == Joint::Afterburner);
    REQUIRE(j.engine == 1);
    s = sim::VehicleState();
    s.engineCount = 2;
    s.throttlePosition[1] = 2.0; // the lever alone lights nothing
    REQUIRE(j.matrix(s)[0][0] == 0.0);
    s.afterburner[1] = 1.0;
    REQUIRE(std::abs(j.matrix(s)[0][0] - 1.0) < 1e-12);
    REQUIRE(Joint::parse("fsim:afterburner", j));
    REQUIRE(j.engine == 0);
    // a leading-edge flap as the flight controls move it, held within its stops
    REQUIRE(Joint::parse("fsim:lef@-2,25", j));
    REQUIRE(j.kind == Joint::LeadingEdge);
    s = sim::VehicleState();
    s.alphaRad = 0.5; // not a guess from the angle of attack
    REQUIRE(std::abs(turn(j.matrix(s))) < 1e-12);
    s.leadingEdgeFlapRad = 10.0 * kRad;
    REQUIRE(std::abs(turn(j.matrix(s)) - 10.0 * kRad) < 1e-9);
    s.leadingEdgeFlapRad = 40.0 * kRad;
    REQUIRE(std::abs(turn(j.matrix(s)) - 25.0 * kRad) < 1e-9);
    REQUIRE(Joint::parse("fsim:lef:-1", j));
    REQUIRE(std::abs(turn(j.matrix(s)) + 40.0 * kRad) < 1e-9);
    // a propeller at its engine's rpm: 600 rpm is a quarter turn in 0.025 s;
    // the turn holds while the clock stops, and does not jump when it jumps
    REQUIRE(Joint::parse("fsim:propeller:0", j));
    REQUIRE(j.kind == Joint::Propeller);
    s = sim::VehicleState();
    s.engineCount = 1;
    s.engineRpm[0] = 600.0;
    s.simTime = 1.0;
    REQUIRE(std::abs(turn(j.matrix(s))) < 1e-12);
    s.simTime = 1.025;
    REQUIRE(std::abs(turn(j.matrix(s)) - 0.5 * 3.14159265358979323846) < 1e-9);
    REQUIRE(std::abs(turn(j.matrix(s)) - 0.5 * 3.14159265358979323846) < 1e-9);
    s.simTime = 50.0;
    REQUIRE(std::abs(turn(j.matrix(s)) - 0.5 * 3.14159265358979323846) < 1e-9);
    s.engineRpm[0] = 0.0; // a stopped engine: a stopped propeller
    s.simTime = 50.5;
    REQUIRE(std::abs(turn(j.matrix(s)) - 0.5 * 3.14159265358979323846) < 1e-9);
    // a nozzle petal by the engine's nozzle: shut, half open
    REQUIRE(Joint::parse("fsim:nozzle:1:8", j));
    REQUIRE(j.kind == Joint::Nozzle);
    s = sim::VehicleState();
    s.engineCount = 2;
    s.throttlePosition[1] = 2.0;
    REQUIRE(std::abs(turn(j.matrix(s))) < 1e-12);
    s.nozzlePosition[1] = 0.5;
    REQUIRE(std::abs(turn(j.matrix(s)) - 4.0 * kRad) < 1e-9);
    REQUIRE_FALSE(Joint::parse("fsim:nozzle:0", j));
    REQUIRE_FALSE(Joint::parse("fsim:propeller:0:10", j));
    REQUIRE_FALSE(Joint::parse("fsim:propeller:7", j));
    REQUIRE_FALSE(Joint::parse("fsim:lef:1:2", j));
    REQUIRE_FALSE(Joint::parse("fsim:lef@25,-2", j));
    REQUIRE_FALSE(Joint::parse("fsim:gear", j));
    REQUIRE_FALSE(Joint::parse("fsim:gear:x", j));
    REQUIRE_FALSE(Joint::parse("fsim:gear:90:0.5:0.5", j));
    REQUIRE_FALSE(Joint::parse("fsim:afterburner:9", j));
    REQUIRE_FALSE(Joint::parse("fsim:elevator@20,-50", j));
    REQUIRE_FALSE(Joint::parse("fsim:elevator@-50", j));
    REQUIRE_FALSE(Joint::parse("fsim:elevator@-50,20x", j));
    REQUIRE_FALSE(Joint::parse("elevator", j));
}
