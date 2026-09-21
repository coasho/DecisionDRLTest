#include "platform/Http.h"

#ifdef _WIN32
#    include <windows.h>
#    include <winhttp.h>
#endif

namespace fsim::platform {

#ifdef _WIN32

namespace {

std::wstring widen(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    std::wstring w(static_cast<std::size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
    return w;
}

struct Handle {
    HINTERNET h = nullptr;
    ~Handle() { if (h) WinHttpCloseHandle(h); }
};

void fail(std::string* error, const std::string& what) {
    if (error) *error = what + " (error " + std::to_string(GetLastError()) + ")";
}

} // namespace

bool httpGet(const std::string& url, std::vector<std::uint8_t>& body, std::string* error, unsigned timeoutMs) {
    body.clear();
    const std::wstring wurl = widen(url);
    URL_COMPONENTS parts{};
    parts.dwStructSize = sizeof(parts);
    wchar_t host[256] = {}, path[2048] = {};
    parts.lpszHostName = host;
    parts.dwHostNameLength = 256;
    parts.lpszUrlPath = path;
    parts.dwUrlPathLength = 2048;
    if (!WinHttpCrackUrl(wurl.c_str(), 0, 0, &parts)) {
        fail(error, "bad URL");
        return false;
    }

    // One session per process: connection reuse across tiles.
    static Handle session;
    if (!session.h) {
        session.h = WinHttpOpen(L"flightsim/0.1", WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
        if (!session.h) {
            fail(error, "WinHttpOpen");
            return false;
        }
    }
    const int t = static_cast<int>(timeoutMs);
    WinHttpSetTimeouts(session.h, t, t, t, t);

    Handle connection;
    connection.h = WinHttpConnect(session.h, host, parts.nPort, 0);
    if (!connection.h) {
        fail(error, "WinHttpConnect");
        return false;
    }
    Handle request;
    request.h = WinHttpOpenRequest(connection.h, L"GET", path, nullptr, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES,
                                   parts.nScheme == INTERNET_SCHEME_HTTPS ? WINHTTP_FLAG_SECURE : 0);
    if (!request.h) {
        fail(error, "WinHttpOpenRequest");
        return false;
    }
    if (!WinHttpSendRequest(request.h, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0) || !WinHttpReceiveResponse(request.h, nullptr)) {
        fail(error, "request failed");
        return false;
    }
    DWORD status = 0, size = sizeof(status);
    WinHttpQueryHeaders(request.h, WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX, &status, &size, WINHTTP_NO_HEADER_INDEX);
    if (status < 200 || status >= 300) {
        if (error) *error = "HTTP " + std::to_string(status);
        return false;
    }
    for (;;) {
        DWORD available = 0;
        if (!WinHttpQueryDataAvailable(request.h, &available)) {
            fail(error, "read failed");
            return false;
        }
        if (available == 0) break;
        const std::size_t offset = body.size();
        body.resize(offset + available);
        DWORD read = 0;
        if (!WinHttpReadData(request.h, body.data() + offset, available, &read)) {
            fail(error, "read failed");
            return false;
        }
        body.resize(offset + read);
    }
    return true;
}

#else

bool httpGet(const std::string&, std::vector<std::uint8_t>&, std::string* error, unsigned) {
    if (error) *error = "httpGet is not implemented on this platform yet";
    return false;
}

#endif

} // namespace fsim::platform
