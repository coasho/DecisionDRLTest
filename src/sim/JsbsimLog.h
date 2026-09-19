#pragma once

namespace fsim::sim {

/// Route JSBSim's global logger into core::Log under category "jsbsim"
/// (design 13). Idempotent; safe to call from several places. JSBSim's logger
/// is process-global and is driven from any thread that steps a vehicle, so
/// the bridge keeps its per-message state thread-local.
void installJsbsimLogBridge();

} // namespace fsim::sim
