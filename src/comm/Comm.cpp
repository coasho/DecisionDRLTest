#include "comm/Comm.h"

#include "core/Geodesy.h"
#include "core/Log.h"
#include "core/Units.h"
#include "platform/Udp.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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

// --- Bridges: a node's traffic over a byte transport ---------------------------------

namespace {

class UdpTransport final : public Transport {
public:
    const char* id() const noexcept override { return "udp"; }
    bool send(const std::uint8_t* bytes, std::size_t length) override { return socket.send(bytes, length); }
    bool receive(std::vector<std::uint8_t>& out) override { return socket.receive(out); }
    platform::UdpSocket socket;
};

constexpr std::size_t kWireHeader = 36;

void put32(std::uint8_t* p, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<std::uint8_t>(v >> (8 * i));
}
std::uint32_t get32(const std::uint8_t* p) {
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(p[i]) << (8 * i);
    return v;
}

} // namespace

std::unique_ptr<Transport> createUdpTransport(std::uint16_t localPort, const std::string& remoteHost, std::uint16_t remotePort, std::string* error) {
    auto t = std::make_unique<UdpTransport>();
    if (!t->socket.open(localPort, remoteHost, remotePort, error)) return nullptr;
    LOG_INFO("comm") << "udp bridge: port " << t->socket.localPort() << " -> " << (remoteHost.empty() ? "127.0.0.1" : remoteHost) << ":" << remotePort;
    return t;
}

std::vector<std::uint8_t> encodeWire(const Message& m) {
    std::vector<std::uint8_t> w(kWireHeader + m.payload.bytes.size());
    std::memcpy(w.data(), "FSMG", 4);
    w[4] = 1;
    put32(&w[8], m.from);
    put32(&w[12], m.to);
    put32(&w[16], m.channel);
    put32(&w[20], m.payload.format);
    static_assert(sizeof(double) == 8, "wire format assumes 8-byte doubles");
    std::memcpy(&w[24], &m.timeSent, 8); // host order: little-endian on every platform this builds on
    put32(&w[32], static_cast<std::uint32_t>(m.payload.bytes.size()));
    if (!m.payload.bytes.empty()) std::memcpy(&w[kWireHeader], m.payload.bytes.data(), m.payload.bytes.size());
    return w;
}

bool decodeWire(const std::uint8_t* bytes, std::size_t length, Message& out) noexcept {
    if (!bytes || length < kWireHeader || std::memcmp(bytes, "FSMG", 4) != 0 || bytes[4] != 1) return false;
    const std::uint32_t n = get32(&bytes[32]);
    if (length < kWireHeader + n) return false;
    out.from = get32(&bytes[8]);
    out.to = get32(&bytes[12]);
    out.channel = get32(&bytes[16]);
    out.payload.format = get32(&bytes[20]);
    std::memcpy(&out.timeSent, &bytes[24], 8);
    out.timeDelivered = 0.0;
    out.payload.bytes.assign(bytes + kWireHeader, bytes + kWireHeader + n);
    return true;
}

void BridgeProtocol::onStep(Node& node, Network& network, const control::WorldView*, double, double) {
    // Everything the peer sent since the last step becomes a message from this node.
    while (transport_->receive(buffer_)) {
        Message m;
        if (!decodeWire(buffer_.data(), buffer_.size(), m)) {
            ++rejected_;
            continue;
        }
        m.from = node.address();
        network.send(std::move(m));
        ++in_;
    }
}

void BridgeProtocol::onReceive(Node&, const Message& message) {
    const auto wire = encodeWire(message);
    if (transport_->send(wire.data(), wire.size())) ++out_;
}

} // namespace fsim::comm
