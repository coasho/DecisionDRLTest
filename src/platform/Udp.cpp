#include "platform/Udp.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <cstring>

namespace fsim::platform {

namespace {

#ifdef _WIN32
using socket_t = SOCKET;
constexpr socket_t kInvalid = INVALID_SOCKET;
struct WinsockInit {
    WinsockInit() { WSADATA d; WSAStartup(MAKEWORD(2, 2), &d); }
    ~WinsockInit() { WSACleanup(); }
};
void ensureWinsock() { static WinsockInit init; }
int lastError() { return WSAGetLastError(); }
void closeSocket(socket_t s) { closesocket(s); }
bool setNonBlocking(socket_t s) { u_long on = 1; return ioctlsocket(s, static_cast<long>(FIONBIO), &on) == 0; }
#else
using socket_t = int;
constexpr socket_t kInvalid = -1;
void ensureWinsock() {}
int lastError() { return errno; }
void closeSocket(socket_t s) { ::close(s); }
bool setNonBlocking(socket_t s) { return fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK) == 0; }
#endif

} // namespace

UdpSocket::~UdpSocket() { close(); }

bool UdpSocket::open(std::uint16_t localPort, const std::string& remoteHost, std::uint16_t remotePort, std::string* error) {
    close();
    ensureWinsock();
    socket_t s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == kInvalid) {
        if (error) *error = "socket() failed: " + std::to_string(lastError());
        return false;
    }
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(localPort);
    if (::bind(s, reinterpret_cast<const sockaddr*>(&local), sizeof local) != 0) {
        if (error) *error = "bind(" + std::to_string(localPort) + ") failed: " + std::to_string(lastError());
        closeSocket(s);
        return false;
    }
    sockaddr_in bound{};
#ifdef _WIN32
    int boundLength = sizeof bound;
#else
    socklen_t boundLength = sizeof bound;
#endif
    if (::getsockname(s, reinterpret_cast<sockaddr*>(&bound), &boundLength) == 0) localPort_ = ntohs(bound.sin_port);

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* result = nullptr;
    const std::string host = remoteHost.empty() ? "127.0.0.1" : remoteHost;
    if (::getaddrinfo(host.c_str(), std::to_string(remotePort).c_str(), &hints, &result) != 0 || !result) {
        if (error) *error = "cannot resolve " + host;
        closeSocket(s);
        return false;
    }
    remoteLength_ = static_cast<int>(result->ai_addrlen);
    std::memcpy(remote_, result->ai_addr, static_cast<std::size_t>(remoteLength_));
    ::freeaddrinfo(result);

    if (!setNonBlocking(s)) {
        if (error) *error = "cannot make the socket non-blocking";
        closeSocket(s);
        return false;
    }
    handle_ = static_cast<long long>(s);
    return true;
}

void UdpSocket::close() {
    if (handle_ != -1) closeSocket(static_cast<socket_t>(handle_));
    handle_ = -1;
    localPort_ = 0;
}

bool UdpSocket::send(const std::uint8_t* bytes, std::size_t length) {
    if (handle_ == -1) return false;
    const auto n = ::sendto(static_cast<socket_t>(handle_), reinterpret_cast<const char*>(bytes), static_cast<int>(length), 0,
                            reinterpret_cast<const sockaddr*>(remote_), remoteLength_);
    return n >= 0 && static_cast<std::size_t>(n) == length;
}

bool UdpSocket::receive(std::vector<std::uint8_t>& out) {
    if (handle_ == -1) return false;
    out.resize(65536);
    const auto n = ::recvfrom(static_cast<socket_t>(handle_), reinterpret_cast<char*>(out.data()), static_cast<int>(out.size()), 0, nullptr, nullptr);
    if (n <= 0) {
        out.clear();
        return false;
    }
    out.resize(static_cast<std::size_t>(n));
    return true;
}

} // namespace fsim::platform
