#include "effects/Effect.h"

#include "sim/FlightModel.h"

namespace fsim::effects {

sim::PropertyHandle EffectContext::property(std::string_view path) { return model_.property(path); }

} // namespace fsim::effects
