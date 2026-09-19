#pragma once

#include "sim/VehicleState.h"

#include <atomic>
#include <cstdint>
#include <vector>

namespace fsim::sim {

/// One published batch of vehicle snapshots.
struct SnapshotBatch {
    double simTime = 0.0;            ///< simulation time of the batch, seconds
    std::int64_t wallNs = 0;         ///< platform::Clock::nanoseconds() at publish
    std::uint64_t sequence = 0;      ///< increments per publish
    std::vector<VehicleState> states;
};

/// Lock-free single-producer / single-consumer triple buffer (design 6.3).
/// The sim thread writes into a free slot and publishes it with one atomic
/// exchange; the render thread acquires the latest published slot the same
/// way. Neither side ever blocks on the other.
class SnapshotBuffer {
public:
    explicit SnapshotBuffer(std::size_t vehicles) {
        for (auto& s : slots_) s.states.resize(vehicles);
    }

    /// Producer: slot to fill. Valid until publish().
    SnapshotBatch& beginWrite() noexcept { return slots_[writeIndex_]; }

    /// Producer: make the written slot the latest.
    void publish() noexcept {
        slots_[writeIndex_].sequence = ++sequence_;
        writeIndex_ = latest_.exchange(writeIndex_, std::memory_order_acq_rel);
    }

    /// Consumer: latest published batch, or nullptr if nothing new since the
    /// last call. The returned pointer stays valid until the next acquire().
    const SnapshotBatch* acquire() noexcept {
        const int idx = latest_.exchange(readIndex_, std::memory_order_acq_rel);
        readIndex_ = idx;
        const SnapshotBatch& b = slots_[readIndex_];
        if (b.sequence == 0 || b.sequence == lastSeen_) return nullptr;
        lastSeen_ = b.sequence;
        return &b;
    }

    /// Consumer: the most recently acquired batch (may be stale), or nullptr
    /// before the first publish.
    const SnapshotBatch* current() const noexcept {
        const SnapshotBatch& b = slots_[readIndex_];
        return b.sequence == 0 ? nullptr : &b;
    }

private:
    SnapshotBatch slots_[3];
    int writeIndex_ = 0;
    int readIndex_ = 2;
    std::atomic<int> latest_{1};
    std::uint64_t sequence_ = 0;
    std::uint64_t lastSeen_ = 0;
};

} // namespace fsim::sim
