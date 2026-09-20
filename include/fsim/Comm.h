#pragma once

// Abstract communication interfaces (design 9.6, ADR-22): nodes exchange
// messages through a medium that decides delivery; codecs give payloads a
// format; protocols add application-level behaviour. Every piece is an
// interface with a small built-in implementation.

#include "fsim/Control.h" // WorldView
#include "fsim/Export.h"
#include "fsim/Rng.h"
#include "fsim/Span.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace fsim::comm {

using Address = std::uint32_t;
inline constexpr Address kBroadcast = 0xFFFFFFFFu;
inline constexpr Address kNoAddress = 0;

/// Payload format ids. Users register their own above kUserFormat.
enum Format : std::uint32_t { kRaw = 1, kJson = 2, kUserFormat = 1000 };

struct Payload {
    std::uint32_t format = kRaw;
    std::vector<std::uint8_t> bytes;
};

struct Message {
    Address from = kNoAddress;
    Address to = kBroadcast;
    std::uint32_t channel = 0;
    double timeSent = 0.0;
    double timeDelivered = 0.0;
    Payload payload;
};

/// Raw codec: trivially copyable structs as bytes (same build both ends).
template <typename T>
Payload encodeRaw(const T& value) {
    static_assert(std::is_trivially_copyable_v<T>, "raw payloads must be trivially copyable");
    Payload p;
    p.format = kRaw;
    p.bytes.resize(sizeof(T));
    std::memcpy(p.bytes.data(), &value, sizeof(T));
    return p;
}
template <typename T>
bool decodeRaw(const Payload& p, T& out) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "raw payloads must be trivially copyable");
    if (p.format != kRaw || p.bytes.size() != sizeof(T)) return false;
    std::memcpy(&out, p.bytes.data(), sizeof(T));
    return true;
}

/// Named numeric fields; the JSON codec is the language-neutral format.
using Fields = std::vector<std::pair<std::string, double>>;
struct FSIM_API JsonCodec {
    static Payload encode(const Fields& fields);
    static bool decode(const Payload& payload, Fields& out);
};

/// A codec plugin for other formats (msgpack, protobuf, ...).
class Codec {
public:
    virtual ~Codec() = default;
    virtual const char* id() const noexcept = 0;
    virtual std::uint32_t format() const noexcept = 0;
    virtual Payload encode(const Fields& fields) const = 0;
    virtual bool decode(const Payload& payload, Fields& out) const = 0;
};

class Network;

/// An endpoint: a vehicle (address = vehicle id) or an external node.
class Node {
public:
    Node(Address address, std::uint32_t vehicleId) : address_(address), vehicleId_(vehicleId) {}
    Address address() const noexcept { return address_; }
    std::uint32_t vehicleId() const noexcept { return vehicleId_; } ///< 0 for external nodes

    /// Messages delivered during the last network step.
    Span<const Message> inbox() const noexcept { return Span<const Message>(inbox_); }
    std::uint64_t sent() const noexcept { return sent_; }
    std::uint64_t received() const noexcept { return received_; }

private:
    friend class Network;
    Address address_;
    std::uint32_t vehicleId_;
    std::vector<Message> inbox_;
    std::uint64_t sent_ = 0, received_ = 0;
};

/// Delivery model: decides if and when a message reaches a node.
class Medium {
public:
    virtual ~Medium() = default;
    virtual const char* id() const noexcept = 0;
    /// Return false to drop; else set `deliverAt` (>= now).
    virtual bool route(const Message& message, const Node& from, const Node& to, const control::WorldView* world, double now,
                       Rng& rng, double& deliverAt) = 0;
};

/// Instant, lossless.
class IdealMedium final : public Medium {
public:
    const char* id() const noexcept override { return "ideal"; }
    bool route(const Message&, const Node&, const Node&, const control::WorldView*, double now, Rng&, double& deliverAt) override {
        deliverAt = now;
        return true;
    }
};

/// Range-limited, delayed, jittered, lossy link between vehicle nodes.
class FSIM_API LinkModel final : public Medium {
public:
    const char* id() const noexcept override { return "link"; }
    bool route(const Message& message, const Node& from, const Node& to, const control::WorldView* world, double now, Rng& rng,
               double& deliverAt) override;

    double rangeM = 20000.0;
    double latencyS = 0.05;
    double jitterS = 0.01;
    double lossProbability = 0.0;
};

/// Application-level behaviour attached to a node (beacons, request/response, ...).
class Protocol {
public:
    virtual ~Protocol() = default;
    virtual const char* id() const noexcept = 0;
    virtual void onStep(Node& node, Network& network, const control::WorldView* world, double simTime, double dt) = 0;
    virtual void onReceive(Node& node, const Message& message) { (void)node; (void)message; }
};

/// Periodic JSON state report on `channel` (id, lat, lon, alt, heading, speed).
class FSIM_API BeaconProtocol final : public Protocol {
public:
    explicit BeaconProtocol(double periodS = 1.0, std::uint32_t channel = 1) : periodS_(periodS), channel_(channel) {}
    const char* id() const noexcept override { return "beacon"; }
    void onStep(Node& node, Network& network, const control::WorldView* world, double simTime, double dt) override;

private:
    double periodS_, nextAt_ = 0.0;
    std::uint32_t channel_;
};

/// The world's message fabric, stepped with the simulation.
class FSIM_API Network {
public:
    Network();
    ~Network();

    Node& createNode(Address address, std::uint32_t vehicleId = 0);
    void removeNode(Address address);
    Node* node(Address address) noexcept;
    const Node* node(Address address) const noexcept;

    void setMedium(std::unique_ptr<Medium> medium) { medium_ = std::move(medium); }
    Medium& medium() noexcept { return *medium_; }
    void attach(Address address, std::unique_ptr<Protocol> protocol);

    /// Queue a message from a node. `timeSent` is stamped here.
    void send(Message message);

    /// Run protocols, route queued messages, deliver due ones into inboxes.
    void step(double simTime, double dt, const control::WorldView* world, Rng& rng);

    std::uint64_t routed() const noexcept { return routed_; }
    std::uint64_t dropped() const noexcept { return dropped_; }

private:
    struct InFlight {
        Message message;
        Address to;
        double deliverAt;
    };
    struct Entry {
        std::unique_ptr<Node> node;
        std::vector<std::unique_ptr<Protocol>> protocols;
    };
    std::vector<Entry> nodes_;
    std::vector<Message> outbox_;
    std::vector<InFlight> inflight_;
    std::unique_ptr<Medium> medium_;
    double now_ = 0.0;
    std::uint64_t routed_ = 0, dropped_ = 0;
};

} // namespace fsim::comm
