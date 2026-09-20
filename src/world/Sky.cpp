#include "world/Sky.h"

#include <chrono>
#include <cmath>
#include <ctime>

namespace fsim::world {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
} // namespace

vsg::dvec3 sunDirectionEcef(int dayOfYear, double utcHours) {
    const double declination = 23.44 * kDeg * std::sin(2.0 * kPi * (284.0 + dayOfYear) / 365.0);
    const double subsolarLon = (180.0 - 15.0 * utcHours) * kDeg; // sun over Greenwich at 12:00 UTC
    return vsg::dvec3(std::cos(declination) * std::cos(subsolarLon), std::cos(declination) * std::sin(subsolarLon),
                      std::sin(declination));
}

vsg::ref_ptr<vsg::Node> createSunLight(int dayOfYear, double utcHours, float sunIntensity, float ambient) {
    auto group = vsg::Group::create();

    auto ambientLight = vsg::AmbientLight::create();
    ambientLight->name = "ambient";
    ambientLight->color.set(1.0f, 1.0f, 1.0f);
    ambientLight->intensity = ambient;
    group->addChild(ambientLight);

    auto sun = vsg::DirectionalLight::create();
    sun->name = "sun";
    sun->color.set(1.0f, 0.98f, 0.94f);
    sun->intensity = sunIntensity;
    const vsg::dvec3 toSun = sunDirectionEcef(dayOfYear, utcHours);
    sun->direction = -toSun; // direction the light travels
    group->addChild(sun);
    return group;
}

void utcOf(double unixSeconds, int& dayOfYear, double& utcHours) {
    const std::time_t t = static_cast<std::time_t>(unixSeconds);
    std::tm utc{};
#ifdef _WIN32
    gmtime_s(&utc, &t);
#else
    gmtime_r(&t, &utc);
#endif
    dayOfYear = utc.tm_yday + 1;
    utcHours = utc.tm_hour + utc.tm_min / 60.0 + utc.tm_sec / 3600.0;
}

void setSunDirection(vsg::Node* sunLight, const vsg::dvec3& toSunEcef) {
    auto* group = dynamic_cast<vsg::Group*>(sunLight);
    if (!group) return;
    for (auto& child : group->children)
        if (auto* sun = dynamic_cast<vsg::DirectionalLight*>(child.get())) sun->direction = -toSunEcef;
}

void currentUtc(int& dayOfYear, double& utcHours) {
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
#ifdef _WIN32
    gmtime_s(&utc, &now);
#else
    gmtime_r(&now, &utc);
#endif
    dayOfYear = utc.tm_yday + 1;
    utcHours = utc.tm_hour + utc.tm_min / 60.0 + utc.tm_sec / 3600.0;
}

} // namespace fsim::world
