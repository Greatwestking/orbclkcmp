#include "get_ant.hpp"

#include <algorithm>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "prn_index.hpp"
#include "standard_reader.hpp"
#include "time_convert.hpp"

namespace orbclkcmp {
namespace {

int Get_Frequency_Index(char system, const std::string& frequency, int& nextIndex, int& currentIndex) {
    if (system == 'E') {
        if (frequency == "E01") currentIndex = 0;
        else if (frequency == "E05") currentIndex = 1;
        else if (frequency == "E07") currentIndex = 2;
        else if (frequency == "E08") currentIndex = 3;
        else if (frequency == "E06") currentIndex = 4;
        return currentIndex;
    }
    if (system == 'C') {
        currentIndex = nextIndex++;
        return currentIndex;
    }
    currentIndex = nextIndex++;
    return currentIndex;
}

int Get_PCO_Frequency_Count(char system, int fileFrequencyCount) {
    int count = std::max(fileFrequencyCount, 3);
    if (system == 'E') {
        count = std::max(count, 5);
    }
    return count;
}

bool Valid_Keyword(const std::string& line, const std::string& keyword) {
    return field(line, 61, 80).find(keyword) != std::string::npos;
}

bool Is_Leap_Year(int year) {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

int Date_To_Year_Doy(int year, int month, int day) {
    static const int monthDays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int doy = day;
    for (int index = 0; index < month - 1; ++index) {
        doy += monthDays[index];
    }
    if (month > 2 && Is_Leap_Year(year)) {
        ++doy;
    }
    return year * 1000 + doy;
}

void Add_BDS_Switch_PRN(BDSSwitchDayPRNs& switchDays, int yearDoy, int prn) {
    if (prn < 1 || prn > constants::CNum) {
        return;
    }
    std::vector<int>& prns = switchDays[yearDoy];
    if (std::find(prns.begin(), prns.end(), prn) == prns.end()) {
        prns.push_back(prn);
        std::sort(prns.begin(), prns.end());
    }
}

}  // namespace

std::array<Antenna, constants::TotalSatNum> Get_Ant(const std::string& path, double mjd) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open antenna file: " + path);
    }

    std::array<Antenna, constants::TotalSatNum> antennas{};
    std::string line;
    while (std::getline(input, line)) {
        if (Valid_Keyword(line, "END OF HEADER")) {
            break;
        }
    }

    while (std::getline(input, line)) {
        if (!Valid_Keyword(line, "START OF ANTENNA")) {
            continue;
        }

        char system = '\0';
        int prn = 0;
        int svn = 0;
        int antennaIndex = -1;
        int frequencyCount = 0;
        int nextFrequencyIndex = 0;
        int currentFrequencyIndex = -1;
        bool satelliteAntenna = false;
        bool blockValid = true;
        int validFromYearDoy = 0;
        int validUntilYearDoy = 0;
        std::string antennaType;

        while (std::getline(input, line)) {
            if (Valid_Keyword(line, "TYPE / SERIAL NO")) {
                antennaType = readText(line, 1, 20);
                system = field(line, 21, 21)[0];
                if (system == ' ') {
                    return antennas;
                }
                if (system != 'G' && system != 'R' && system != 'C' && system != 'E'
                    && system != 'J') {
                    satelliteAntenna = false;
                    continue;
                }
                prn = readInt(line, 22, 23);
                antennaIndex = Get_PRNIndex(system, prn);
                if (antennaIndex < 0) {
                    satelliteAntenna = false;
                    continue;
                }
                try {
                    svn = readInt(line, 42, 44);
                } catch (const std::exception&) {
                    svn = 0;
                }
                satelliteAntenna = true;
            } else if (Valid_Keyword(line, "# OF FREQUENCIES") && satelliteAntenna) {
                frequencyCount = Get_PCO_Frequency_Count(system, readInt(line, 1, 6));
                nextFrequencyIndex = 0;
            } else if (Valid_Keyword(line, "VALID FROM") && satelliteAntenna) {
                const int year = readInt(line, 1, 6);
                const int month = readInt(line, 7, 12);
                const int day = readInt(line, 13, 18);
                validFromYearDoy = Date_To_Year_Doy(year, month, day);
                const double validFrom = UTC2MJD(year, month, day, readInt(line, 19, 24),
                                                readInt(line, 25, 30), readDouble(line, 31, 43));
                if (mjd < validFrom) {
                    blockValid = false;
                }
            } else if (Valid_Keyword(line, "VALID UNTIL") && satelliteAntenna) {
                const int year = readInt(line, 1, 6);
                const int month = readInt(line, 7, 12);
                const int day = readInt(line, 13, 18);
                validUntilYearDoy = Date_To_Year_Doy(year, month, day);
                const double validUntil = UTC2MJD(year, month, day, readInt(line, 19, 24),
                                                 readInt(line, 25, 30), readDouble(line, 31, 43));
                if (mjd > validUntil) {
                    blockValid = false;
                }
            } else if (Valid_Keyword(line, "START OF FREQUENCY") && satelliteAntenna && blockValid) {
                if (frequencyCount <= 0) {
                    frequencyCount = Get_PCO_Frequency_Count(system, 0);
                }
                Antenna& antenna = antennas[static_cast<std::size_t>(antennaIndex)];
                if (antenna.pco.frequencyCount() == 0) {
                    antenna.pco.resize(frequencyCount);
                    antenna.frequency.resize(static_cast<std::size_t>(frequencyCount));
                }
                antenna.antennaType = antennaType;
                antenna.prn = prn;
                antenna.svn = svn;
                antenna.validFromYearDoy = validFromYearDoy;
                antenna.validUntilYearDoy = validUntilYearDoy;

                const std::string frequencyName = readText(line, 1, 6);
                const int freqIndex = Get_Frequency_Index(system, frequencyName, nextFrequencyIndex,
                                                           currentFrequencyIndex);
                if (freqIndex >= 0 && freqIndex < static_cast<int>(antenna.frequency.size())) {
                    antenna.frequency[static_cast<std::size_t>(freqIndex)] = frequencyName;
                }
                while (std::getline(input, line)) {
                    if (Valid_Keyword(line, "NORTH / EAST / UP")) {
                        if (freqIndex >= 0 && freqIndex < antenna.pco.frequencyCount()) {
                            antenna.pco.at(freqIndex) = {
                                readDouble(line, 1, 10) / 1000.0,
                                readDouble(line, 11, 20) / 1000.0,
                                readDouble(line, 21, 30) / 1000.0,
                            };
                        }
                    } else if (Valid_Keyword(line, "END OF FREQUENCY")) {
                        break;
                    }
                }
            }

            if (Valid_Keyword(line, "END OF ANTENNA")) {
                break;
            }
        }
    }

    return antennas;
}

BDSSwitchDayPRNs Read_BDS_Switch_Day_PRNs(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open antenna file: " + path);
    }

    BDSSwitchDayPRNs switchDays;
    std::string line;
    while (std::getline(input, line)) {
        if (Valid_Keyword(line, "END OF HEADER")) {
            break;
        }
    }

    while (std::getline(input, line)) {
        if (!Valid_Keyword(line, "START OF ANTENNA")) {
            continue;
        }

        bool bdsAntenna = false;
        int prn = 0;
        while (std::getline(input, line)) {
            if (Valid_Keyword(line, "TYPE / SERIAL NO")) {
                const char system = field(line, 21, 21)[0];
                bdsAntenna = system == 'C';
                if (bdsAntenna) {
                    prn = readInt(line, 22, 23);
                }
            } else if (bdsAntenna && (Valid_Keyword(line, "VALID FROM")
                       || Valid_Keyword(line, "VALID UNTIL"))) {
                const int year = readInt(line, 1, 6);
                const int month = readInt(line, 7, 12);
                const int day = readInt(line, 13, 18);
                Add_BDS_Switch_PRN(switchDays, Date_To_Year_Doy(year, month, day), prn);
            }

            if (Valid_Keyword(line, "END OF ANTENNA")) {
                break;
            }
        }
    }

    return switchDays;
}

}  // namespace orbclkcmp
