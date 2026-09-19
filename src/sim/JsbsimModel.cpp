#include "sim/JsbsimModel.h"

#include "core/Log.h"
#include "core/Units.h"
#include "sim/GroundProvider.h"
#include "sim/JsbsimLog.h"

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <input_output/FGGroundCallback.h>
#include <input_output/FGPropertyManager.h>
#include <math/FGColumnVector3.h>
#include <math/FGLocation.h>
#include <math/FGMatrix33.h>
#include <math/FGQuaternion.h>
#include <models/FGAccelerations.h>
#include <models/FGAuxiliary.h>
#include <models/FGFCS.h>
#include <models/FGGroundReactions.h>
#include <models/FGInertial.h>
#include <models/FGOutput.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/propulsion/FGEngine.h>
#include <simgear/misc/sg_path.hxx>

#include <algorithm>
#include <cmath>

namespace fsim::sim {

namespace {

/// JSBSim ground callback backed by a GroundProvider (design 7.3). Modelled on
/// FGDefaultGroundCallback but with the terrain height taken from the provider
/// at the queried geodetic position. Feet in, feet out (JSBSim internals).
class ProviderGroundCallback final : public JSBSim::FGGroundCallback {
public:
    ProviderGroundCallback(std::shared_ptr<const GroundProvider> provider, double semiMajorFt, double semiMinorFt)
        : provider_(std::move(provider)), a_(semiMajorFt), b_(semiMinorFt) {}

    double GetAGLevel(double, const JSBSim::FGLocation& loc, JSBSim::FGLocation& contact,
                      JSBSim::FGColumnVector3& normal, JSBSim::FGColumnVector3& vel,
                      JSBSim::FGColumnVector3& angularVel) const override {
        vel.InitMatrix();
        angularVel.InitMatrix();

        JSBSim::FGLocation l = loc;
        l.SetEllipse(a_, b_);
        const double lat = l.GetGeodLatitudeRad();
        const double lon = l.GetLongitude();
        const double cosLat = std::cos(lat);
        normal = JSBSim::FGColumnVector3(cosLat * std::cos(lon), cosLat * std::sin(lon), std::sin(lat));

        const double terrainFt = units::metresToFeet(provider_->heightAboveEllipsoidM(lat, lon));
        contact.SetEllipse(a_, b_);
        contact.SetPositionGeodetic(lon, lat, terrainFt);
        return l.GetGeodAltitude() - terrainFt;
    }

    // JSBSim may try to move the terrain (IC files, scripts); the provider is
    // authoritative, so these are deliberately ignored.
    void SetTerrainElevation(double) override {}
    void SetEllipse(double semimajor, double semiminor) override {
        a_ = semimajor;
        b_ = semiminor;
    }

private:
    std::shared_ptr<const GroundProvider> provider_;
    double a_, b_;
};

bool finite(double v) noexcept { return std::isfinite(v); }

#ifdef _WIN32
constexpr const char* kNullDevice = "NUL";
#else
constexpr const char* kNullDevice = "/dev/null";
#endif

} // namespace

JsbsimModel::JsbsimModel(double dt, std::shared_ptr<const GroundProvider> ground)
    : dt_(dt), ground_(std::move(ground)) {
    installJsbsimLogBridge();

    // Quiet JSBSim's own console chatter before the first instance prints its banner.
    JSBSim::FGJSBBase::debug_lvl = 0;

    fdm_ = std::make_unique<JSBSim::FGFDMExec>();
    fdm_->SetDebugLevel(0);
    fdm_->Setdt(dt_);
    fdm_->DisableOutput();

    auto inertial = fdm_->GetInertial();
    inertial->SetGroundCallback(
        new ProviderGroundCallback(ground_, inertial->GetSemimajor(), inertial->GetSemiminor()));
}

JsbsimModel::~JsbsimModel() = default;

bool JsbsimModel::load(const AircraftSpec& aircraft, const InitialConditions& ic) {
    loaded_ = false;
    diverged_ = false;
    stepCount_ = 0;

    const SGPath root(aircraft.jsbsimRoot.string());
    fdm_->SetRootDir(root);
    fdm_->SetAircraftPath(SGPath("aircraft"));
    fdm_->SetEnginePath(SGPath("engine"));
    fdm_->SetSystemsPath(SGPath("systems"));

    try {
        if (!fdm_->LoadModel(aircraft.name)) {
            LOG_ERROR("sim") << "JSBSim failed to load aircraft '" << aircraft.name << "' from " << aircraft.jsbsimRoot.string();
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("sim") << "JSBSim exception loading '" << aircraft.name << "': " << e.what();
        return false;
    } catch (...) {
        LOG_ERROR("sim") << "JSBSim unknown exception loading '" << aircraft.name << "'";
        return false;
    }

    // Aircraft files may declare their own <output> (CSV/socket). The platform
    // owns logging and recording, so those are disabled - and, because JSBSim
    // opens output files in RunIC() even when disabled, redirected to the null
    // device so no stray files appear in the working directory.
    {
        auto output = fdm_->GetOutput();
        for (unsigned i = 0; !output->GetOutputName(i).empty(); ++i) output->SetOutputName(i, kNullDevice);
        fdm_->DisableOutput();
    }
    cacheCommandNodes();

    try {
        applyInitialConditions(ic);
        if (!fdm_->RunIC()) {
            LOG_ERROR("sim") << "JSBSim RunIC failed for '" << aircraft.name << "'";
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("sim") << "JSBSim exception in initial conditions: " << e.what();
        return false;
    }

    loaded_ = true;
    LOG_DEBUG("sim") << "loaded '" << aircraft.name << "', " << fdm_->GetPropulsion()->GetNumEngines()
                     << " engine(s), dt=" << dt_;
    return true;
}

bool JsbsimModel::reset(const InitialConditions& ic) {
    if (!loaded_) return false;
    diverged_ = false;
    stepCount_ = 0;
    try {
        applyInitialConditions(ic);
        // Mode 0: reinitialise models and run the IC pass (design 7.2).
        fdm_->ResetToInitialConditions(0);
    } catch (const std::exception& e) {
        LOG_ERROR("sim") << "JSBSim exception on reset: " << e.what();
        return false;
    }
    return true;
}

void JsbsimModel::applyInitialConditions(const InitialConditions& ic) {
    auto IC = fdm_->GetIC();
    IC->SetGeodLatitudeDegIC(ic.latitudeDeg); // geodetic; SetLatitudeDegIC would be geocentric
    IC->SetLongitudeDegIC(ic.longitudeDeg);
    IC->SetPsiDegIC(ic.headingDeg);
    IC->SetThetaDegIC(ic.pitchDeg);
    IC->SetPhiDegIC(ic.rollDeg);
    IC->SetVtrueKtsIC(units::metresPerSecondToKnots(ic.airspeedTrueMs));

    if (ic.onGround) {
        // Let JSBSim place the gear on the provider's terrain.
        const double terrainM = ground_->heightAboveEllipsoidM(units::degreesToRadians(ic.latitudeDeg),
                                                               units::degreesToRadians(ic.longitudeDeg));
        IC->SetTerrainElevationFtIC(units::metresToFeet(terrainM));
        IC->SetAltitudeAGLFtIC(0.0);
    } else {
        IC->SetAltitudeASLFtIC(units::metresToFeet(ic.altitudeMslM));
    }
}

void JsbsimModel::cacheCommandNodes() {
    auto pm = fdm_->GetPropertyManager();
    auto node = [&](const char* path) { return PropertyHandle(pm->GetNode(path, true)); };

    aileronCmd_ = node("fcs/aileron-cmd-norm");
    elevatorCmd_ = node("fcs/elevator-cmd-norm");
    rudderCmd_ = node("fcs/rudder-cmd-norm");
    flapCmd_ = node("fcs/flap-cmd-norm");
    gearCmd_ = node("gear/gear-cmd-norm");
    leftBrakeCmd_ = node("fcs/left-brake-cmd-norm");
    rightBrakeCmd_ = node("fcs/right-brake-cmd-norm");

    throttleCmd_.clear();
    const auto engines = std::min<std::size_t>(fdm_->GetPropulsion()->GetNumEngines(), ControlInputs::kMaxEngines);
    for (std::size_t i = 0; i < engines; ++i) {
        throttleCmd_.push_back(PropertyHandle(pm->GetNode("fcs/throttle-cmd-norm", static_cast<int>(i), true)));
    }
}

void JsbsimModel::step(const ControlInputs& in) {
    if (!loaded_ || diverged_) return;

    aileronCmd_.set(in.aileron);
    elevatorCmd_.set(in.elevator);
    rudderCmd_.set(in.rudder);
    flapCmd_.set(in.flaps);
    gearCmd_.set(in.gearDown);
    leftBrakeCmd_.set(in.brakeLeft);
    rightBrakeCmd_.set(in.brakeRight);
    for (std::size_t i = 0; i < throttleCmd_.size(); ++i) throttleCmd_[i].set(in.throttle[i]);

    fdm_->Run();
    ++stepCount_;

    checkDivergence();
}

bool JsbsimModel::checkDivergence() {
    // A NaN anywhere in position/velocity poisons everything downstream;
    // mark the vehicle so the environment terminates and resets it (design 13).
    const auto prop = fdm_->GetPropagate();
    const auto& loc = prop->GetLocation();
    const auto& uvw = prop->GetUVW();
    const bool ok = finite(loc(1)) && finite(loc(2)) && finite(loc(3)) && finite(uvw(1)) && finite(uvw(2)) &&
                    finite(uvw(3)) && finite(prop->GetAltitudeASL());
    if (!ok && !diverged_) {
        diverged_ = true;
        LOG_WARN("sim") << "vehicle diverged (non-finite state) at step " << stepCount_;
    }
    return !diverged_;
}

void JsbsimModel::state(VehicleState& out) const {
    using namespace units;
    out = VehicleState{};
    out.simTime = fdm_->GetSimTime();
    out.stepCount = stepCount_;
    out.diverged = diverged_;
    if (!loaded_) return;

    const auto prop = fdm_->GetPropagate();
    const auto aux = fdm_->GetAuxiliary();
    const auto fcs = fdm_->GetFCS();
    const auto accel = fdm_->GetAccelerations();
    const auto propulsion = fdm_->GetPropulsion();

    const JSBSim::FGLocation& loc = prop->GetLocation();
    for (int i = 0; i < 3; ++i) out.positionEcef[i] = feetToMetres(loc(static_cast<unsigned>(i + 1)));

    const JSBSim::FGQuaternion q = prop->GetQuaternionECEF(); // ECEF -> body
    for (int i = 0; i < 4; ++i) out.attitudeEcefToBody[i] = q(static_cast<unsigned>(i + 1));

    out.latitudeRad = loc.GetGeodLatitudeRad();
    out.longitudeRad = loc.GetLongitude();
    out.altitudeMslM = feetToMetres(prop->GetAltitudeASL());
    out.altitudeAglM = feetToMetres(prop->GetDistanceAGL());

    const auto& euler = prop->GetEuler();
    const auto& uvw = prop->GetUVW();
    const auto& ned = prop->GetVel();
    const auto& pqr = prop->GetPQR();
    const auto& uvwDot = accel->GetUVWdot();
    for (unsigned i = 1; i <= 3; ++i) {
        out.eulerRad[i - 1] = euler(i);
        out.velocityBodyMs[i - 1] = feetToMetres(uvw(i));
        out.velocityNedMs[i - 1] = feetToMetres(ned(i));
        out.angularRateBodyRadS[i - 1] = pqr(i);
        out.accelerationBodyMs2[i - 1] = feetToMetres(uvwDot(i));
    }

    out.airspeedTrueMs = feetToMetres(aux->GetVtrueFPS());
    out.airspeedCalibratedMs = knotsToMetresPerSecond(aux->GetVcalibratedKTS());
    out.mach = aux->GetMach();
    out.alphaRad = aux->Getalpha();
    out.betaRad = aux->Getbeta();
    out.loadFactor = aux->GetNlf();

    out.aileronRad = fcs->GetDaLPos(JSBSim::ofRad);
    out.elevatorRad = fcs->GetDePos(JSBSim::ofRad);
    out.rudderRad = fcs->GetDrPos(JSBSim::ofRad);
    out.flapsRad = fcs->GetDfPos(JSBSim::ofRad);
    out.gearPosition = fcs->GetGearPos();

    const auto engines = std::min<std::size_t>(propulsion->GetNumEngines(), VehicleState::kMaxEngines);
    out.engineCount = static_cast<int>(engines);
    for (std::size_t i = 0; i < engines; ++i) {
        out.throttlePosition[i] = fcs->GetThrottlePos(static_cast<int>(i));
        out.thrustN[i] = poundsForceToNewtons(propulsion->GetEngine(static_cast<unsigned>(i))->GetThrust());
    }
    out.fuelKg = propulsion->GetTanksWeight() * 0.45359237; // tank contents, lbs -> kg

    out.onGround = fdm_->GetGroundReactions()->GetWOW();

    const JSBSim::FGMatrix33& tb2ec = prop->GetTb2ec(); // body -> ECEF
    for (unsigned r = 1; r <= 3; ++r)
        for (unsigned c = 1; c <= 3; ++c) out.rotationBodyToEcef[(r - 1) * 3 + (c - 1)] = tb2ec(r, c);
}

PropertyHandle JsbsimModel::property(std::string_view path) {
    auto* node = fdm_->GetPropertyManager()->GetNode(std::string(path), false);
    if (!node) LOG_WARN("sim") << "unknown JSBSim property '" << path << "'";
    return PropertyHandle(node);
}

} // namespace fsim::sim
