#include "sim/JsbsimModel.h"

#include "core/Log.h"
#include "core/Units.h"
#include "sim/GroundProvider.h"
#include "sim/JsbsimLog.h"

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <initialization/FGTrim.h>
#include <input_output/FGGroundCallback.h>
#include <input_output/FGPropertyManager.h>
#include <input_output/FGXMLElement.h>
#include <math/FGColumnVector3.h>
#include <math/FGLocation.h>
#include <math/FGMatrix33.h>
#include <math/FGQuaternion.h>
#include <models/FGAccelerations.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>
#include <models/FGExternalReactions.h>
#include <models/atmosphere/FGWinds.h>
#include <models/FGFCS.h>
#include <models/FGGroundReactions.h>
#include <models/FGInertial.h>
#include <models/FGOutput.h>
#include <models/FGPropagate.h>
#include <models/FGPropulsion.h>
#include <models/propulsion/FGEngine.h>
#include <models/propulsion/FGThruster.h>
#include <models/propulsion/FGTurbine.h>
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

    // JSBSim asks once per contact point per step. The geodetic conversion
    // (iterative) is done for the first point of a step only; the others -
    // metres away, on the same aircraft - are placed in its local east-north-up
    // frame, which is exact to d^2 / 2R (under 0.1 mm within kLocalFt). The
    // terrain is still sampled under every point.
    double GetAGLevel(double t, const JSBSim::FGLocation& loc, JSBSim::FGLocation& contact,
                      JSBSim::FGColumnVector3& normal, JSBSim::FGColumnVector3& vel,
                      JSBSim::FGColumnVector3& angularVel) const override {
        vel.InitMatrix();
        angularVel.InitMatrix();

        const double x = loc(1), y = loc(2), z = loc(3);
        double lat, lon, alt;
        const double dx = x - x0_, dy = y - y0_, dz = z - z0_;
        if (t == t0_ && dx * dx + dy * dy + dz * dz < kLocalFt * kLocalFt) {
            const double e = -sinLon0_ * dx + cosLon0_ * dy;
            const double n = -sinLat0_ * cosLon0_ * dx - sinLat0_ * sinLon0_ * dy + cosLat0_ * dz;
            const double u = cosLat0_ * cosLon0_ * dx + cosLat0_ * sinLon0_ * dy + sinLat0_ * dz;
            lat = lat0_ + n / feetPerRadLat_;
            lon = lon0_ + e / feetPerRadLon_;
            alt = alt0_ + u;
        } else {
            JSBSim::FGLocation l = loc;
            l.SetEllipse(a_, b_);
            lat = l.GetGeodLatitudeRad();
            lon = l.GetLongitude();
            alt = l.GetGeodAltitude();
            t0_ = t;
            x0_ = x, y0_ = y, z0_ = z;
            lat0_ = lat, lon0_ = lon, alt0_ = alt;
            sinLat0_ = std::sin(lat), cosLat0_ = std::cos(lat), sinLon0_ = std::sin(lon), cosLon0_ = std::cos(lon);
            // radii of curvature (feet): meridian M and prime vertical N, at the point's height
            const double e2 = 1.0 - (b_ * b_) / (a_ * a_);
            const double w = std::sqrt(1.0 - e2 * sinLat0_ * sinLat0_);
            feetPerRadLat_ = a_ * (1.0 - e2) / (w * w * w) + alt;
            feetPerRadLon_ = (a_ / w + alt) * cosLat0_;
        }
        const double cosLat = std::cos(lat);
        normal = JSBSim::FGColumnVector3(cosLat * std::cos(lon), cosLat * std::sin(lon), std::sin(lat));

        const double terrainFt = units::metresToFeet(provider_->heightAboveEllipsoidM(lat, lon));
        contact.SetEllipse(a_, b_);
        contact.SetPositionGeodetic(lon, lat, terrainFt);
        return alt - terrainFt;
    }

    // JSBSim may try to move the terrain (IC files, scripts); the provider is
    // authoritative, so these are deliberately ignored.
    void SetTerrainElevation(double) override {}
    void SetEllipse(double semimajor, double semiminor) override {
        a_ = semimajor;
        b_ = semiminor;
    }

private:
    static constexpr double kLocalFt = 100.0; // points this close to a step's first share its frame

    std::shared_ptr<const GroundProvider> provider_;
    double a_, b_;
    // The step's first point (ECEF feet) and its local frame; one vehicle's
    // callback is only ever called from the thread stepping that vehicle.
    mutable double t0_ = -1.0, x0_ = 0.0, y0_ = 0.0, z0_ = 0.0;
    mutable double lat0_ = 0.0, lon0_ = 0.0, alt0_ = 0.0;
    mutable double sinLat0_ = 0.0, cosLat0_ = 1.0, sinLon0_ = 0.0, cosLon0_ = 1.0;
    mutable double feetPerRadLat_ = 1.0, feetPerRadLon_ = 1.0;
};

bool finite(double v) noexcept { return std::isfinite(v); }

constexpr double kGroundSpawnCgHeightM = 1.5; // CG start height for on-ground spawns before ground trim

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
    fdm_->SetAircraftPath(aircraft.aircraftDir.empty() ? SGPath("aircraft") : SGPath(aircraft.aircraftDir.string()));
    fdm_->SetEnginePath(SGPath("engine"));
    fdm_->SetSystemsPath(SGPath("systems"));

    try {
        if (!fdm_->LoadModel(aircraft.name)) {
            LOG_ERROR("sim") << "JSBSim failed to load aircraft '" << aircraft.name << "' from "
                             << (aircraft.aircraftDir.empty() ? aircraft.jsbsimRoot / "aircraft" : aircraft.aircraftDir).string();
            return false;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("sim") << "JSBSim exception loading '" << aircraft.name << "': " << e.what();
        return false;
    } catch (...) {
        LOG_ERROR("sim") << "JSBSim unknown exception loading '" << aircraft.name << "'";
        return false;
    }

    silenceOutputs();
    installExternalReaction();
    cacheCommandNodes();

    try {
        applyInitialConditions(ic);
        if (!fdm_->RunIC()) {
            LOG_ERROR("sim") << "JSBSim RunIC failed for '" << aircraft.name << "'";
            return false;
        }
        startEngines();
        settleOnGround(ic);
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
        // RunIC() reopens every output file; JSBSim only closes them when a
        // new output is started, so close them first (keeping the null name)
        // or the reopen fails and is logged on every reset.
        silenceOutputs();
        fdm_->GetOutput()->SetStartNewOutput();
        // JSBSim's reset keeps a propeller's RPM (FGPropeller::ResetToIC
        // clears only the induced velocity): after a blow-up the reset pass
        // would compute thrust from it. Start the propellers as a load does.
        auto propulsion = fdm_->GetPropulsion();
        for (unsigned i = 0; i < propulsion->GetNumEngines(); ++i)
            if (auto* thruster = propulsion->GetEngine(i)->GetThruster()) thruster->SetRPM(0.0);
        // Mode 0: reinitialise models and run the IC pass (design 7.2).
        fdm_->ResetToInitialConditions(0);
        startEngines();
        settleOnGround(ic);
        seedIntegrators();
    } catch (const std::exception& e) {
        LOG_ERROR("sim") << "JSBSim exception on reset: " << e.what();
        return false;
    }
    return true;
}

// The integrators (Adams-Bashforth) remember past accelerations. RunIC()
// seeds that memory from what Propagate read at the start of its pass: the
// last step's accelerations before a reset, since Accelerations runs after
// Propagate. After a violent step or a blow-up, the first step after the
// reset replayed them - and blew up again. Evaluate the models once at the
// new state, without integrating, and seed it from that (as JSBSim's own
// SetHoldDown() does).
void JsbsimModel::seedIntegrators() {
    fdm_->SuspendIntegration();
    fdm_->Run();
    fdm_->ResumeIntegration();
    auto propagate = fdm_->GetPropagate();
    const auto accelerations = fdm_->GetAccelerations();
    propagate->in.vPQRidot = accelerations->GetPQRidot();
    propagate->in.vUVWidot = accelerations->GetUVWidot();
    propagate->InitializeDerivatives();
}

// Aircraft files may declare their own <output> (CSV/socket). The platform
// owns logging and recording, so those are disabled - and, because JSBSim
// opens output files in RunIC() even when disabled, redirected to the null
// device so no stray files appear in the working directory.
void JsbsimModel::silenceOutputs() {
    auto output = fdm_->GetOutput();
    for (unsigned i = 0; !output->GetOutputName(i).empty(); ++i) output->SetOutputName(i, kNullDevice);
    fdm_->DisableOutput();
}

// A generic body-frame force and moment at the CG, defined in memory so that
// effects can push on any stock aircraft without editing its XML (design 9.5).
void JsbsimModel::installExternalReaction() {
    auto leaf = [](const char* name, const char* value) {
        auto* e = new JSBSim::Element(name);
        e->AddData(value);
        return e;
    };
    auto triplet = [&](const char* name, const char* x, const char* y, const char* z) {
        auto* e = new JSBSim::Element(name);
        e->AddChildElement(leaf("x", x));
        e->AddChildElement(leaf("y", y));
        e->AddChildElement(leaf("z", z));
        return e;
    };
    // Elements are reference counted (SGSharedPtr): JSBSim's loader takes and
    // drops references while parsing, so hold the root ourselves.
    JSBSim::Element_ptr reactions = new JSBSim::Element("external_reactions");
    auto* force = new JSBSim::Element("force");
    force->AddAttribute("name", "fsim");
    force->AddAttribute("frame", "BODY");
    auto* location = triplet("location", "0", "0", "0");
    location->AddAttribute("unit", "IN");
    force->AddChildElement(location);
    force->AddChildElement(triplet("direction", "1", "0", "0"));
    reactions->AddChildElement(force);
    auto* moment = new JSBSim::Element("moment");
    moment->AddAttribute("name", "fsim");
    moment->AddAttribute("frame", "BODY");
    moment->AddChildElement(triplet("direction", "1", "0", "0"));
    reactions->AddChildElement(moment);
    try {
        // An aircraft with its own <external_reactions> (f16: pushback, hook) has
        // already tied the summary properties; Load() ties them again and logs an
        // error for each, so untie first (the same object re-ties them).
        auto pm = fdm_->GetPropertyManager();
        for (const char* tied : {"moments/l-external-lbsft", "moments/m-external-lbsft", "moments/n-external-lbsft",
                                 "forces/fbx-external-lbs", "forces/fby-external-lbs", "forces/fbz-external-lbs"})
            if (pm->HasNode(tied)) pm->Untie(tied);
        fdm_->GetExternalReactions()->Load(reactions);
    } catch (const std::exception& e) {
        LOG_WARN("sim") << "external reaction not installed: " << e.what();
    }
    extForceMag_ = property("external_reactions/fsim/magnitude");
    extForceX_ = property("external_reactions/fsim/x");
    extForceY_ = property("external_reactions/fsim/y");
    extForceZ_ = property("external_reactions/fsim/z");
    extMomentMag_ = property("external_reactions/fsim/magnitude-lbsft");
    extMomentL_ = property("external_reactions/fsim/l");
    extMomentM_ = property("external_reactions/fsim/m");
    extMomentN_ = property("external_reactions/fsim/n");
    extLocX_ = property("external_reactions/fsim/location-x-in");
    extLocY_ = property("external_reactions/fsim/location-y-in");
    extLocZ_ = property("external_reactions/fsim/location-z-in");
    cgX_ = property("inertia/cg-x-in");
    cgY_ = property("inertia/cg-y-in");
    cgZ_ = property("inertia/cg-z-in");
    extForceMag_.set(0.0);
    extMomentMag_.set(0.0);
}

// Engines running with full mixture from the first step (an airborne start
// with a dead engine is never what a scenario means); the throttle is the
// caller's from the next step on.
void JsbsimModel::startEngines() {
    auto propulsion = fdm_->GetPropulsion();
    if (propulsion->GetNumEngines() == 0) return;
    try {
        propulsion->InitRunning(-1);
    } catch (const std::exception& e) {
        LOG_WARN("sim") << "engine start: " << e.what();
    }
}

void JsbsimModel::setWindNed(double north, double east, double down) {
    if (!loaded_) return;
    fdm_->GetWinds()->SetWindNED(units::metresToFeet(north), units::metresToFeet(east), units::metresToFeet(down));
}

void JsbsimModel::setTurbulence(double intensity, double windSpeed20ftMs) {
    if (!loaded_) return;
    auto winds = fdm_->GetWinds();
    if (intensity <= 0.0) {
        winds->SetTurbType(JSBSim::FGWinds::ttNone);
        return;
    }
    // Milspec (Dryden) turbulence: severity index 0..7 (3 light, 4 moderate, 6 severe).
    winds->SetTurbType(JSBSim::FGWinds::ttMilspec);
    winds->SetProbabilityOfExceedence(std::clamp(static_cast<int>(std::lround(1.0 + intensity * 6.0)), 1, 7));
    winds->SetWindspeed20ft(units::metresToFeet(std::max(windSpeed20ftMs, 1.0)));
}

void JsbsimModel::setAtmosphere(double temperatureSeaLevelK, double pressureSeaLevelPa) {
    if (!loaded_) return;
    auto atmosphere = fdm_->GetAtmosphere();
    atmosphere->SetTemperatureSL(temperatureSeaLevelK, JSBSim::FGAtmosphere::eKelvin);
    atmosphere->SetPressureSL(JSBSim::FGAtmosphere::ePascals, pressureSeaLevelPa);
}

void JsbsimModel::setExternalForceBody(const double forceN[3], const double momentNm[3]) {
    if (!loaded_ || !extForceMag_.valid()) return;
    // At the centre of gravity (structural frame, inches), so a pure force makes no moment.
    extLocX_.set(cgX_.get());
    extLocY_.set(cgY_.get());
    extLocZ_.set(cgZ_.get());
    const double f = std::sqrt(forceN[0] * forceN[0] + forceN[1] * forceN[1] + forceN[2] * forceN[2]);
    if (f > 1e-9) {
        extForceX_.set(forceN[0] / f);
        extForceY_.set(forceN[1] / f);
        extForceZ_.set(forceN[2] / f);
    }
    extForceMag_.set(f * units::kNewtonsToPoundsForce);
    const double m = std::sqrt(momentNm[0] * momentNm[0] + momentNm[1] * momentNm[1] + momentNm[2] * momentNm[2]);
    if (m > 1e-9) {
        extMomentL_.set(momentNm[0] / m);
        extMomentM_.set(momentNm[1] / m);
        extMomentN_.set(momentNm[2] / m);
    }
    constexpr double kNewtonMetresToPoundFeet = units::kNewtonsToPoundsForce * units::kMetresToFeet;
    extMomentMag_.set(m * kNewtonMetresToPoundFeet);
}

void JsbsimModel::seed(std::uint64_t value) {
    fdm_->GetRandomGenerator()->seed(static_cast<unsigned int>(value ^ (value >> 32)));
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
        // Start with the CG a gear-height above the provider's terrain; the
        // ground trim after RunIC() then settles the gear onto the local slope.
        // (AGL 0 would bury the gear and launch the aircraft off its springs.)
        const double terrainM = ground_->heightAboveEllipsoidM(units::degreesToRadians(ic.latitudeDeg),
                                                               units::degreesToRadians(ic.longitudeDeg));
        IC->SetTerrainElevationFtIC(units::metresToFeet(terrainM));
        IC->SetAltitudeAGLFtIC(units::metresToFeet(kGroundSpawnCgHeightM));
    } else {
        IC->SetAltitudeASLFtIC(units::metresToFeet(ic.altitudeMslM));
    }
}

void JsbsimModel::settleOnGround(const InitialConditions& ic) {
    if (!ic.onGround) return;
    // JSBSim's ground trim: adjusts altitude, pitch and roll until the gear
    // forces balance the weight on the terrain under each wheel.
    try {
        fdm_->DoTrim(JSBSim::tGround);
    } catch (const std::exception& e) {
        LOG_WARN("sim") << "ground trim failed (" << e.what() << "); the vehicle will settle dynamically";
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

    // read-only: an aircraft whose FCS has no leading-edge flaps reports none
    lefPosDeg_ = PropertyHandle(pm->GetNode("fcs/lef-pos-deg", false));

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
    const double altM = units::feetToMetres(prop->GetAltitudeASL());
    const double speedMs = units::feetToMetres(uvw.Magnitude());
    // Non-finite, or physically impossible for an aircraft: catches the onset of
    // a numerical blow-up a few steps in instead of after the values overflow.
    const bool ok = finite(loc(1)) && finite(loc(2)) && finite(loc(3)) && finite(uvw(1)) && finite(uvw(2)) &&
                    finite(uvw(3)) && finite(altM) && speedMs < 5000.0 && altM > -1000.0 && altM < 200000.0;
    if (!ok && !diverged_) {
        diverged_ = true;
        LOG_WARN("sim") << "vehicle diverged (non-finite state) at step " << stepCount_ << "; last good state: lat "
                        << lastGood_.latDeg << " lon " << lastGood_.lonDeg << " alt " << lastGood_.altM << " m, agl "
                        << lastGood_.aglM << " m, |v| " << lastGood_.speedMs << " m/s, on ground " << lastGood_.onGround;
    } else if (ok) {
        // Keep a compact copy of the last finite state for the divergence report.
        lastGood_.latDeg = prop->GetGeodLatitudeDeg();
        lastGood_.lonDeg = prop->GetLongitudeDeg();
        lastGood_.altM = altM;
        lastGood_.aglM = units::feetToMetres(prop->GetDistanceAGL());
        lastGood_.speedMs = speedMs;
        lastGood_.onGround = fdm_->GetGroundReactions()->GetWOW();
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
        const auto engine = propulsion->GetEngine(static_cast<unsigned>(i));
        out.thrustN[i] = poundsForceToNewtons(engine->GetThrust());
        out.engineRpm[i] = engine->GetThruster() ? engine->GetThruster()->GetRPM() : 0.0;
        out.engineN2[i] = out.afterburner[i] = out.nozzlePosition[i] = 0.0;
        if (engine->GetType() == JSBSim::FGEngine::etTurbine) {
            const auto* turbine = static_cast<const JSBSim::FGTurbine*>(engine.get());
            out.engineN2[i] = turbine->GetN2();
            out.nozzlePosition[i] = turbine->GetNozzle();
            // lit: in full (a throttle-gated afterburner), or by how far the
            // lever runs past 1 (a staged one, JSBSim's augmethod 2)
            if (turbine->GetAugmentation())
                out.afterburner[i] = out.throttlePosition[i] > 1.0 ? std::min(out.throttlePosition[i] - 1.0, 1.0) : 1.0;
        }
    }
    out.leadingEdgeFlapRad = lefPosDeg_.valid() ? lefPosDeg_.get() * 3.14159265358979323846 / 180.0 : 0.0;
    // the wheels: each strut's compression, its steering, the wheel's roll
    const auto& ground = fdm_->GetGroundReactions();
    out.wheelCount = 0;
    for (int i = 0; i < ground->GetNumGearUnits() && out.wheelCount < VehicleState::kMaxWheels; ++i) {
        const auto unit = ground->GetGearUnit(i);
        if (!unit->IsBogey()) continue;
        const int k = out.wheelCount++;
        out.wheelCompressionM[k] = feetToMetres(unit->GetCompLen());
        out.wheelSteerRad[k] = unit->GetSteerAngleDeg() * 3.14159265358979323846 / 180.0;
        out.wheelSpeedMs[k] = feetToMetres(unit->GetWheelRollVel());
    }
    for (int k = out.wheelCount; k < VehicleState::kMaxWheels; ++k)
        out.wheelCompressionM[k] = out.wheelSteerRad[k] = out.wheelSpeedMs[k] = 0.0;
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
