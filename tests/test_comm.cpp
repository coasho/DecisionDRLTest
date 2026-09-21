// Communication bridges: a node's traffic over UDP in the "FSMG" wire format,
// with a plain socket standing in for the external process (design 9.6).
#include "comm/Comm.h"

#include "core/Rng.h"
#include "platform/Udp.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

using namespace fsim;

namespace {

bool receiveWithin(platform::UdpSocket& s, std::vector<std::uint8_t>& out, int ms) {
    for (int i = 0; i < ms; ++i) {
        if (s.receive(out)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

/// A port nobody is bound to right now (bind to 0, read it back, release it).
std::uint16_t freePort() {
    platform::UdpSocket probe;
    REQUIRE(probe.open(0, "127.0.0.1", 1));
    return probe.localPort();
}

} // namespace

TEST_CASE("wire format round trip", "[comm]") {
    comm::Message m;
    m.from = 7;
    m.to = comm::kBroadcast;
    m.channel = 3;
    m.timeSent = 12.5;
    m.payload = comm::JsonCodec::encode({{"x", 1.5}, {"y", -2.0}});
    const auto wire = comm::encodeWire(m);
    REQUIRE(wire.size() == 36 + m.payload.bytes.size());
    comm::Message back;
    REQUIRE(comm::decodeWire(wire.data(), wire.size(), back));
    REQUIRE(back.from == 7);
    REQUIRE(back.to == comm::kBroadcast);
    REQUIRE(back.channel == 3);
    REQUIRE(back.timeSent == 12.5);
    REQUIRE(back.payload.format == comm::kJson);
    REQUIRE(back.payload.bytes == m.payload.bytes);
    REQUIRE_FALSE(comm::decodeWire(wire.data(), 20, back));              // truncated header
    REQUIRE_FALSE(comm::decodeWire(wire.data(), wire.size() - 1, back)); // truncated payload
    std::vector<std::uint8_t> junk(40, 0);
    REQUIRE_FALSE(comm::decodeWire(junk.data(), junk.size(), back));
}

TEST_CASE("udp bridge carries a node's traffic both ways", "[comm][udp]") {
    const std::uint16_t bridgePort = freePort();
    // The "external process": a socket on its own port, talking to the bridge's.
    platform::UdpSocket peer;
    std::string error;
    REQUIRE(peer.open(0, "127.0.0.1", bridgePort, &error));
    auto transport = comm::createUdpTransport(bridgePort, "127.0.0.1", peer.localPort(), &error);
    REQUIRE(transport);
    REQUIRE(comm::createUdpTransport(bridgePort, "127.0.0.1", 1, &error) == nullptr); // port taken
    REQUIRE(error.find("bind") != std::string::npos);

    comm::Network net;
    net.createNode(1, 1); // a vehicle
    net.createNode(2);    // the bridged node: the peer's presence in the world
    auto* bridge = new comm::BridgeProtocol(std::move(transport));
    net.attach(2, std::unique_ptr<comm::Protocol>(bridge));
    Rng rng(1);
    double t = 0.0;

    // Outbound: vehicle 1 -> node 2 leaves as one datagram.
    comm::Message m;
    m.from = 1;
    m.to = 2;
    m.channel = 5;
    m.payload = comm::JsonCodec::encode({{"cmd", 42.0}});
    net.send(m);
    net.step(t += 0.01, 0.01, nullptr, rng);
    REQUIRE(bridge->datagramsOut() == 1);
    std::vector<std::uint8_t> datagram;
    REQUIRE(receiveWithin(peer, datagram, 2000));
    comm::Message got;
    REQUIRE(comm::decodeWire(datagram.data(), datagram.size(), got));
    REQUIRE(got.from == 1);
    REQUIRE(got.to == 2);
    REQUIRE(got.channel == 5);
    REQUIRE(got.timeSent == 0.0); // stamped when queued, before the first step
    comm::Fields fields;
    REQUIRE(comm::JsonCodec::decode(got.payload, fields));
    REQUIRE(fields.at(0).first == "cmd");
    REQUIRE(fields.at(0).second == 42.0);

    // Inbound: the peer addresses vehicle 1 on channel 7 (whatever `from` it claims, the world sees node 2).
    comm::Message reply;
    reply.from = 999;
    reply.to = 1;
    reply.channel = 7;
    reply.payload = comm::encodeRaw(3.25);
    const auto wire = comm::encodeWire(reply);
    REQUIRE(peer.send(wire.data(), wire.size()));
    const std::uint8_t junk[3] = {1, 2, 3};
    REQUIRE(peer.send(junk, sizeof junk)); // not a wire message: counted, ignored
    bool delivered = false;
    for (int i = 0; i < 2000 && !delivered; ++i) {
        net.step(t += 0.01, 0.01, nullptr, rng);
        delivered = !net.node(1)->inbox().empty();
        if (!delivered) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    REQUIRE(delivered);
    const auto& in = net.node(1)->inbox()[0];
    REQUIRE(in.from == 2);
    REQUIRE(in.to == 1);
    REQUIRE(in.channel == 7);
    double value = 0.0;
    REQUIRE(comm::decodeRaw(in.payload, value));
    REQUIRE(value == 3.25);
    REQUIRE(bridge->datagramsIn() == 1);
    REQUIRE(bridge->rejected() == 1);
    // A message from the bridge never comes back out through it.
    REQUIRE(bridge->datagramsOut() == 1);
}
