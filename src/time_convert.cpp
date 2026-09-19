#include "time_convert.hpp"

#include <cmath>
#include <stdexcept>

namespace orbclkcmp {

namespace {

int Normalize_Year(int year) {
    if (year < 80) {
        return year + 2000;
    }
    if (year >= 80 && year < 100) {
        return year + 1900;
    }
    return year;
}

}  // namespace

Gpst Day2GPST(int year, int doy) {
    if (year < 1901 || year > 2099) {
        throw std::out_of_range("Day2GPST currently supports years 1901..2099");
    }
    if (doy < 1 || doy > 366) {
        throw std::out_of_range("day-of-year must be in 1..366");
    }

    int days = (year - 1980) * 365 + doy - 5;
    days += (year - 1980) / 4;
    if ((year - 1980) % 4 == 0) {
        days -= 1;
    }

    return Gpst{days / 7, days % 7};
}

double UTC2MJD(int year, int month, int day, int hour, int minute, double second) {
    int normalizedYear = Normalize_Year(year);
    int normalizedMonth = month;

    if (normalizedMonth <= 2) {
        normalizedYear -= 1;
        normalizedMonth += 12;
    }

    return std::trunc(365.25 * normalizedYear) +
           std::trunc(30.6001 * static_cast<double>(normalizedMonth + 1)) +
           day +
           (static_cast<double>(hour) + (static_cast<double>(minute) + second / 60.0) / 60.0) / 24.0 +
           1720981.5 -
           2400000.5;
}

GpstTime UTC2GPST(int year, int month, int day, int hour, int minute, double second) {
    const double mjd = UTC2MJD(year, month, day, hour, minute, second);
    const int gpsWeek = static_cast<int>((mjd - 44244.0) / 7.0);
    const double gpsSeconds = std::round((mjd - 44244.0 - gpsWeek * 7.0) * 86400.0 * 1000.0) / 1000.0;
    return GpstTime{gpsWeek, gpsSeconds};
}

}  // namespace orbclkcmp
