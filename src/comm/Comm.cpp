#include "comm/Comm.h"

#include "core/Geodesy.h"
#include "core/Units.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace fsim::comm {

// --- JSON codec (flat object of numbers; enough for state reports and commands) ---

Payload JsonCodec::encode(const Fields& fields) {
    std::string s = "{";
    char buf[64];
    for (std::size_t i = 0; i < fields.size(); ++i) {
        if (i) s += ",";
        s += "\"";
        for (char c : fields[i].first) {
            if (c == '"' || c == '\\') s += '\\';
            s += c;
        }
        s += "\":";
        std::snprintf(buf, sizeof buf, "%.17g", fields[i].second);
        s += buf;
    }
    s += "}";
    Payload p;
    p.format = kJson;
    p.bytes.assign(s.begin(), s.end());
    return p;
}

bool JsonCodec::decode(const Payload& payload, Fields& out) {
    out.clear();
    if (payload.format != kJson) return false;
    const std::string s(payload.bytes.begin(), payload.bytes.end());
    std::size_t i = s.find('{');
    if (i == std::string::npos) return false;
    ++i;
    while (i < s.size()) {
        while (i < s.size() && (s[i] == ' ' || s[i] == ',' || s[i] == '\n')) ++i;
        if (i >= s.size() || s[i] == '}') break;
        if (s[i] != '"') return false;
        std::string key;
        for (++i; i < s.size() && s[i] != '"'; ++i) {
            if (s[i] == '\\' && i + 1 < s.size()) ++i;
            key += s[i];
        }
        if (i >= s.size()) return false;
        ++i;
        while (i < s.size() && (s[i] == ' ' || s[i] == ':')) ++i;
        char* end = nullptr;
        const double v = std::strtod(s.c_str() + i, &end);
        if (end == s.c_str() + i) return false;
        i = static_cast<std::size_t>(end - s.c_str());
        out.emplace_back(std::move(key), v);
    }
    return true;
}

// --- LinkModel ---------------------------------------------------------------------

bool LinkModel::route(const Message&, const Node& from, const Node& to, const control::WorldView* world, double now, Rng& rng,
                      double& deliverAt) {
    if (world && from.vehicleId() && to.vehicleId()) {
        const auto* a = world->vehicleState(from.vehicleId());
        const auto* b = world->vehicleState(to.vehicleId());
        if (a && b) {
            const double ground = geo::distanceM(a->latitudeRad, a->longitudeRad, b->latitudeRad, b->longitudeRad);
            const double distance = std::hypot(ground, a->altitudeMslM - b->altitudeMslM);
            if (distance > rangeM) return false;
        }
    }
    if (lossProbability > 0.0 && rng.uniform() < lossProbability) return false;
    deliverAt = now + std::max(0.0, latencyS + (jitterS > 0.0 ? rng.uniform(-jitterS, jitterS) : 0.0));
    return true;
}

// --- BeaconProtocol ------------------------------------------------------------------

void BeaconProtocol::onStep(Node& node, Network& network, const control::WorldView* world, double simTime, double) {
    if (simTime < nextAt_) return;
    nextAt_ = simTime + periodS_;
    const auto* s = world ? world->vehicleState(node.vehicleId()) : nullptr;
    if (!s) return;
    Message m;
    m.from = node.address();
    m.to = kBroadcast;
    m.channel = channel_;
    m.payload = JsonCodec::encode({{"id", static_cast<double>(node.vehicleId())},
                                   {"lat_deg", units::radiansToDegrees(s->latitudeRad)},
                                   {"lon_deg", units::radiansToDegrees(s->longitudeRad)},
                                   {"alt_m", s->altitudeMslM},
                                   {"heading_deg", units::radiansToDegrees(s->eulerRad[2])},
                                   {"speed_ms", s->airspeedTrueMs}});
    network.send(std::move(m));
}

// --- Network -------------------------------------------------------------------------

Network::Network() : medium_(std::make_unique<IdealMedium>()) {}
Network::~Network() = default;

Node& Network::createNode(Address address, std::uint32_t vehicleId) {
    for (auto& e : nodes_)
        if (e.node->address() == address) return *e.node;
    Entry e;
    e.node = std::make_unique<Node>(address, vehicleId);
    nodes_.push_back(std::move(e));
    return *nodes_.back().node;
}

void Network::removeNode(Address address) {
    nodes_.erase(std::remove_if(nodes_.begin(), nodes_.end(), [&](const Entry& e) { return e.node->address() == address; }), nodes_.end());
}

Node* Network::node(Address address) noexcept {
    for (auto& e : nodes_)
        if (e.node->address() == address) return e.node.get();
    return nullptr;
}

const Node* Network::node(Address address) const noexcept {
    for (const auto& e : nodes_)
        if (e.node->address() == address) return e.node.get();
    return nullptr;
}

void Network::attach(Address address, std::unique_ptr<Protocol> protocol) {
    for (auto& e : nodes_)
        if (e.node->address() == address) {
            e.protocols.push_back(std::move(protocol));
            return;
        }
}

void Network::send(Message message) {
    message.timeSent = now_;
    if (Node* n = node(message.from)) ++n->sent_;
    outbox_.push_back(std::move(message));
}

void Network::step(double simTime, double dt, const control::WorldView* world, Rng& rng) {
    now_ = simTime;
    for (auto& e : nodes_) e.node->inbox_.clear();

    for (auto& e : nodes_)
        for (auto& p : e.protocols) p->onStep(*e.node, *this, world, simTime, dt);

    // Route: fan out broadcasts, ask the medium for a delivery time per receiver.
    for (auto& m : outbox_) {
        const Node* from = node(m.from);
        for (auto& e : nodes_) {
            const Node& to = *e.node;
            if (to.address() == m.from) continue;
            if (m.to != kBroadcast && m.to != to.address()) continue;
            double at = simTime;
            if (from && !medium_->route(m, *from, to, world, simTime, rng, at)) {
                ++dropped_;
                continue;
            }
            inflight_.push_back(InFlight{m, to.address(), at});
            ++routed_;
        }
    }
    outbox_.clear();

    // Deliver what is due, oldest first.
    std::stable_sort(inflight_.begin(), inflight_.end(), [](const InFlight& a, const InFlight& b) { return a.deliverAt < b.deliverAt; });
    std::size_t delivered = 0;
    for (; delivered < inflight_.size() && inflight_[delivered].deliverAt <= simTime; ++delivered) {
        InFlight& f = inflight_[delivered];
        for (auto& e : nodes_)
            if (e.node->address() == f.to) {
                f.message.timeDelivered = simTime;
                for (auto& p : e.protocols) p->onReceive(*e.node, f.message);
                e.node->inbox_.push_back(std::move(f.message));
                ++e.node->received_;
                break;
            }
    }
    inflight_.erase(inflight_.begin(), inflight_.begin() + static_cast<std::ptrdiff_t>(delivered));
}

} // namespace fsim::comm
