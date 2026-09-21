#pragma once

// Civil (proleptic Gregorian, UTC) <-> Unix seconds without the C library's
// timezone state (Howard Hinnant's days_from_civil).

namespace fsim::core {

inline double unixSecondsFromCivil(int year, int month, int day, int hour = 0, int minute = 0, double second = 0.0) noexcept {
    int y = year;
    const int m = month, d = day;
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153u * static_cast<unsigned>(m + (m > 2 ? -3 : 9)) + 2u) / 5u + static_cast<unsigned>(d) - 1u;
    const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    const long long days = static_cast<long long>(era) * 146097LL + static_cast<long long>(doe) - 719468LL;
    return static_cast<double>(days) * 86400.0 + hour * 3600.0 + minute * 60.0 + second;
}

} // namespace fsim::core
