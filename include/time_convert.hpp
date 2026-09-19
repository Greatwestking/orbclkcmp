#pragma once

namespace orbclkcmp {

struct Gpst {
    int week = 0;
    int day = 0;
};

struct GpstTime {
    int week = 0;
    double seconds = 0.0;
};

// Convert year and day-of-year to GPS week and day.
Gpst Day2GPST(int year, int doy);

// Convert UTC calendar fields to modified Julian date.
double UTC2MJD(int year, int month, int day, int hour, int minute, double second);

// Convert UTC calendar fields to GPS week and seconds-of-week.
GpstTime UTC2GPST(int year, int month, int day, int hour, int minute, double second);

}  // namespace orbclkcmp
