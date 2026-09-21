// An external process taking part in a world's network (design 9.6): it
// receives the "FSMG" datagrams a BridgeProtocol sends (beacons of every
// vehicle when used with `multi_level_control --bridge 47000:47001`) and
// answers each one with a message to that vehicle. Only the wire format
// (fsim::comm::encodeWire / decodeWire) is shared with the platform; the
// socket is plain Winsock, as it would be in any language.
//
//   udp_peer [--listen 47001] [--send 47000] [--seconds 30]

#include <fsim/Comm.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char** argv) {
    unsigned listenPort = 47001, sendPort = 47000;
    double seconds = 30.0;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : "0"; };
        if (k == "--listen") listenPort = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--send") sendPort = static_cast<unsigned>(std::atoi(next()));
        else if (k == "--seconds") seconds = std::atof(next());
    }

    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    const SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(static_cast<u_short>(listenPort));
    if (bind(s, reinterpret_cast<const sockaddr*>(&local), sizeof local) != 0) {
        std::fprintf(stderr, "cannot bind udp port %u\n", listenPort);
        return 1;
    }
    u_long nonBlocking = 1;
    ioctlsocket(s, static_cast<long>(FIONBIO), &nonBlocking);
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = htons(static_cast<u_short>(sendPort));
    inet_pton(AF_INET, "127.0.0.1", &peer.sin_addr);
    std::printf("udp_peer: listening on %u, answering to 127.0.0.1:%u\n", listenPort, sendPort);

    std::vector<std::uint8_t> buffer(65536);
    const auto t0 = std::chrono::steady_clock::now();
    unsigned long long received = 0, answered = 0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds) {
        const int n = recvfrom(s, reinterpret_cast<char*>(buffer.data()), static_cast<int>(buffer.size()), 0, nullptr, nullptr);
        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        fsim::comm::Message m;
        if (!fsim::comm::decodeWire(buffer.data(), static_cast<std::size_t>(n), m)) continue;
        ++received;
        fsim::comm::Fields fields;
        if (m.payload.format == fsim::comm::kJson && fsim::comm::JsonCodec::decode(m.payload, fields)) {
            std::printf("from %u ch %u t=%.1f:", m.from, m.channel, m.timeSent);
            for (const auto& [key, value] : fields) std::printf(" %s=%.4g", key.c_str(), value);
            std::printf("\n");
        } else {
            std::printf("from %u ch %u: %zu byte(s), format %u\n", m.from, m.channel, m.payload.bytes.size(), m.payload.format);
        }
        // Answer the sender on channel 9; the world delivers it from the bridge node.
        fsim::comm::Message reply;
        reply.to = m.from;
        reply.channel = 9;
        reply.payload = fsim::comm::JsonCodec::encode({{"ack", static_cast<double>(received)}});
        const auto wire = fsim::comm::encodeWire(reply);
        if (sendto(s, reinterpret_cast<const char*>(wire.data()), static_cast<int>(wire.size()), 0, reinterpret_cast<const sockaddr*>(&peer), sizeof peer) > 0)
            ++answered;
    }
    std::printf("udp_peer: %llu message(s) received, %llu answered\n", received, answered);
    closesocket(s);
    WSACleanup();
    return 0;
}
