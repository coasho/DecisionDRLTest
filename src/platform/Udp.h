#pragma once

// Non-blocking UDP socket (Winsock on Windows, BSD sockets elsewhere): the
// transport under comm bridges. No threads; the caller polls receive().

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace fsim::platform {

class UdpSocket {
public:
    UdpSocket() = default;
    ~UdpSocket();
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    /// Bind to `localPort` (0 = any) on all interfaces and set the default
    /// destination. Returns false (with `error`) when the port is taken or the
    /// host cannot be resolved.
    bool open(std::uint16_t localPort, const std::string& remoteHost, std::uint16_t remotePort, std::string* error = nullptr);
    void close();
    bool isOpen() const noexcept { return handle_ != -1; }
    std::uint16_t localPort() const noexcept { return localPort_; }

    /// One datagram to the default destination; false when the socket is closed or the send failed.
    bool send(const std::uint8_t* bytes, std::size_t length);
    /// Next queued datagram (up to 64 KiB); false when none is waiting.
    bool receive(std::vector<std::uint8_t>& out);

private:
    long long handle_ = -1;
    std::uint16_t localPort_ = 0;
    unsigned char remote_[128] = {};
    int remoteLength_ = 0;
};

} // namespace fsim::platform
