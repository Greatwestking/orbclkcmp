#include "read_nav.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "constants.hpp"
#include "standard_reader.hpp"
#include "time_convert.hpp"

namespace orbclkcmp {
namespace {

int Max_PRN_For_System(char system) {
    switch (system) {
        case 'G':
            return constants::GNum;
        case 'C':
            return constants::CNum;
        case 'E':
            return constants::NumE;
        case 'J':
            return constants::JNum;
        default:
            throw std::invalid_argument("unsupported navigation system");
    }
}

int Normalize_BDS_PRN(int prn) {
    return prn >= 59 ? prn - 12 : prn;
}

double Read_Nav_Value(const std::string& line, int firstColumn, int index) {
    const int start = firstColumn + index * 19;
    return readDouble(line, start, start + 18);
}

bool Read_Line(std::istream& input, std::string& line) {
    return static_cast<bool>(std::getline(input, line));
}

bool Header_Matches(char system, const std::string& line, bool IsCNAV) {
    if (field(line, 1, 7) == "> ION G") {
        return false;
    }
    const std::string recordHead = field(line, 1, 7);
    if (system == 'G') {
        return (!IsCNAV && recordHead == "> EPH G" && line.find("LNAV") != std::string::npos)
            || (IsCNAV && recordHead == "> EPH G" && line.find("CNAV") != std::string::npos
                && line.find("CNAV2") == std::string::npos);
    }
    if (system == 'C') {
        return (!IsCNAV && recordHead == "> EPH C" && line.find(" D") != std::string::npos)
            || (IsCNAV && recordHead == "> EPH C" && line.find("CNV1") != std::string::npos);
    }
    if (system == 'J') {
        return (!IsCNAV && recordHead == "> EPH J" && line.find("LNAV") != std::string::npos)
            || (IsCNAV && recordHead == "> EPH J" && line.find("CNAV") != std::string::npos);
    }
    if (system == 'E') {
        return recordHead == "> EPH E";
    }
    return false;
}

bool Is_Usable_Record(const NavRecord& record, bool requireEccentricity) {
    if (record.weekNo == 0.0 || record.sqrtA == 0.0) {
        return false;
    }
    return !requireEccentricity || record.e != 0.0;
}

void Keep_Epoch_Replacement(std::vector<NavRecordEntry>& entries, const NavRecordEntry& next,
                            bool v4CoverOlderThanPrevious) {
    if (!entries.empty()) {
        const NavRecordEntry& previous = entries.back();
        const double nextTime = next.record.gpsWeek * 604800.0 + next.record.gpsSec;
        const double previousTime = previous.record.gpsWeek * 604800.0 + previous.record.gpsSec;
        const double dt = nextTime - previousTime;
        if (std::abs(dt) < 0.1 && previous.record.health != 0.0) {
            entries.back() = next;
            return;
        }
        if (v4CoverOlderThanPrevious && entries.size() > 1) {
            const NavRecordEntry& previous2 = entries[entries.size() - 2];
            const double previous2Time = previous2.record.gpsWeek * 604800.0 + previous2.record.gpsSec;
            if (nextTime < previous2Time) {
                return;
            }
        } else if (nextTime < previousTime) {
            entries.push_back(next);
            std::iter_swap(entries.end() - 1, entries.end() - 2);
            return;
        }
        if (std::abs(dt) < 0.1 && next.system == 'C'
            && next.record.health == 0.0 && next.record.iode <= 1.0) {
            entries.back() = next;
            return;
        }
    }
    entries.push_back(next);
}

NavRecordEntry Read_First_Nav_Line(const std::string& line, char system, int version, int& year,
                                int& month, int& day, int& hour, int& minute, double& second) {
    int prn = 0;
    const int firstClockColumn = version == 2 ? 23 : 24;
    if (version == 2) {
        prn = readInt(line, 1, 2);
        year = readInt(line, 3, 5);
        month = readInt(line, 6, 8);
        day = readInt(line, 9, 11);
        hour = readInt(line, 12, 14);
        minute = readInt(line, 15, 17);
        second = readDouble(line, 18, 22);
    } else {
        prn = readInt(line, 2, 3);
        year = readInt(line, 4, 8);
        month = readInt(line, 9, 11);
        day = readInt(line, 12, 14);
        hour = readInt(line, 15, 17);
        minute = readInt(line, 18, 20);
        second = readDouble(line, 21, 23);
    }
    if (system == 'C') {
        prn = Normalize_BDS_PRN(prn);
    }

    const auto gpst = UTC2GPST(year, month, day, hour, minute, second);
    NavRecordEntry entry;
    entry.system = system;
    entry.prn = prn;
    entry.record.gpsWeek = gpst.week;
    entry.record.gpsSec = gpst.seconds;
    entry.record.a0 = readDouble(line, firstClockColumn, firstClockColumn + 18);
    entry.record.a1 = readDouble(line, firstClockColumn + 19, firstClockColumn + 37);
    entry.record.a2 = readDouble(line, firstClockColumn + 38, firstClockColumn + 56);
    return entry;
}

bool Read_Ordinary_Nav_Block(std::istream& input, NavRecordEntry& entry, int version) {
    std::string line;
    const int firstColumn = version == 2 ? 4 : 5;
    if (!Read_Line(input, line)) return false;
    entry.record.iode = Read_Nav_Value(line, firstColumn, 0);
    entry.record.crs = Read_Nav_Value(line, firstColumn, 1);
    entry.record.deltaN = Read_Nav_Value(line, firstColumn, 2);
    entry.record.m0 = Read_Nav_Value(line, firstColumn, 3);

    if (!Read_Line(input, line)) return false;
    entry.record.cuc = Read_Nav_Value(line, firstColumn, 0);
    entry.record.e = Read_Nav_Value(line, firstColumn, 1);
    entry.record.cus = Read_Nav_Value(line, firstColumn, 2);
    entry.record.sqrtA = Read_Nav_Value(line, firstColumn, 3);

    if (!Read_Line(input, line)) return false;
    entry.record.toe = Read_Nav_Value(line, firstColumn, 0);
    entry.record.cic = Read_Nav_Value(line, firstColumn, 1);
    entry.record.omega0 = Read_Nav_Value(line, firstColumn, 2);
    entry.record.cis = Read_Nav_Value(line, firstColumn, 3);

    if (!Read_Line(input, line)) return false;
    entry.record.i0 = Read_Nav_Value(line, firstColumn, 0);
    entry.record.crc = Read_Nav_Value(line, firstColumn, 1);
    entry.record.omega = Read_Nav_Value(line, firstColumn, 2);
    entry.record.omegaDot = Read_Nav_Value(line, firstColumn, 3);

    if (!Read_Line(input, line)) return false;
    entry.record.idot = Read_Nav_Value(line, firstColumn, 0);
    if (entry.system == 'E') {
        entry.record.code = Read_Nav_Value(line, firstColumn, 1);
        entry.record.weekNo = Read_Nav_Value(line, firstColumn, 2);
    } else {
        entry.record.weekNo = readDouble(line, firstColumn + 38, firstColumn + 56);
    }

    if (!Read_Line(input, line)) return false;
    entry.record.health = readDouble(line, firstColumn + 19, firstColumn + 37);
    entry.record.tgd[0] = readDouble(line, firstColumn + 38, firstColumn + 56);
    if (entry.system == 'C' || entry.system == 'E') {
        entry.record.tgd[1] = readDouble(line, firstColumn + 57, firstColumn + 75);
    }

    if (!Read_Line(input, line)) return false;
    if (entry.system == 'C') {
        entry.record.iodc = readDouble(line, firstColumn + 19, firstColumn + 37);
        if (entry.record.toe > 604800.0) {
            entry.record.toe -= 604800.0;
            entry.record.weekNo += 1.0;
        }
    }
    return true;
}

bool Read_CNAV_Block(std::istream& input, NavRecordEntry& entry, int version) {
    std::string line;
    const int firstColumn = 5;
    if (!Read_Line(input, line)) return false;
    entry.record.aDot = Read_Nav_Value(line, firstColumn, 0);
    entry.record.crs = Read_Nav_Value(line, firstColumn, 1);
    entry.record.deltaN = Read_Nav_Value(line, firstColumn, 2);
    entry.record.m0 = Read_Nav_Value(line, firstColumn, 3);

    if (!Read_Line(input, line)) return false;
    entry.record.cuc = Read_Nav_Value(line, firstColumn, 0);
    entry.record.e = Read_Nav_Value(line, firstColumn, 1);
    entry.record.cus = Read_Nav_Value(line, firstColumn, 2);
    entry.record.sqrtA = Read_Nav_Value(line, firstColumn, 3);

    if (!Read_Line(input, line)) return false;
    entry.record.toe = Read_Nav_Value(line, firstColumn, 0);
    entry.record.cic = Read_Nav_Value(line, firstColumn, 1);
    entry.record.omega0 = Read_Nav_Value(line, firstColumn, 2);
    entry.record.cis = Read_Nav_Value(line, firstColumn, 3);
    if (entry.system == 'G') {
        entry.record.toe = entry.record.gpsSec;
    }
    entry.record.weekNo = entry.record.gpsWeek;

    if (!Read_Line(input, line)) return false;
    entry.record.i0 = Read_Nav_Value(line, firstColumn, 0);
    entry.record.crc = Read_Nav_Value(line, firstColumn, 1);
    entry.record.omega = Read_Nav_Value(line, firstColumn, 2);
    entry.record.omegaDot = Read_Nav_Value(line, firstColumn, 3);

    if (!Read_Line(input, line)) return false;
    entry.record.idot = Read_Nav_Value(line, firstColumn, 0);
    entry.record.nDot = Read_Nav_Value(line, firstColumn, 1);

    if (entry.system == 'C') {
        if (!Read_Line(input, line)) return false;
        if (!Read_Line(input, line)) return false;
        entry.record.tgd[0] = readDouble(line, 43, 61);
        entry.record.tgd[1] = readDouble(line, 62, 80);
        if (!Read_Line(input, line)) return false;
        entry.record.health = readDouble(line, 24, 42);
        entry.record.iodc = readDouble(line, 62, 80);
        if (!Read_Line(input, line)) return false;
        entry.record.iode = readDouble(line, 62, 80);
    } else {
        if (!Read_Line(input, line)) return false;
        entry.record.health = readDouble(line, 24, 42);
        entry.record.tgd[0] = readDouble(line, 43, 61);
        if (!Read_Line(input, line)) return false;
        entry.record.iscL1Ca = Read_Nav_Value(line, firstColumn, 0);
        entry.record.iscL2C = Read_Nav_Value(line, firstColumn, 1);
        entry.record.iscL5I5 = Read_Nav_Value(line, firstColumn, 2);
        entry.record.iscL5Q5 = Read_Nav_Value(line, firstColumn, 3);
        if (!Read_Line(input, line)) return false;
    }
    (void)version;
    return true;
}

void Read_V4_Ion_Block(std::istream& input, NavHead& head) {
    std::string line;
    if (!Read_Line(input, line)) return;
    head.alpha[0] = readDouble(line, 24, 42);
    head.alpha[1] = readDouble(line, 43, 61);
    head.alpha[2] = readDouble(line, 62, 80);
    if (!Read_Line(input, line)) return;
    head.alpha[3] = readDouble(line, 5, 23);
    head.beta[0] = readDouble(line, 24, 42);
    head.beta[1] = readDouble(line, 43, 61);
    head.beta[2] = readDouble(line, 62, 80);
    if (!Read_Line(input, line)) return;
    head.beta[3] = readDouble(line, 5, 23);
}

GlonassNavRecordEntry Read_First_Glonass_Line(const std::string& line, int version) {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    int prn = 0;
    const int firstClockColumn = version == 2 ? 23 : 24;
    if (version == 2) {
        prn = readInt(line, 1, 2);
        year = readInt(line, 3, 5);
        month = readInt(line, 6, 8);
        day = readInt(line, 9, 11);
        hour = readInt(line, 12, 14);
        minute = readInt(line, 15, 17);
        second = readDouble(line, 18, 22);
    } else {
        prn = readInt(line, 2, 3);
        year = readInt(line, 4, 8);
        month = readInt(line, 9, 11);
        day = readInt(line, 12, 14);
        hour = readInt(line, 15, 17);
        minute = readInt(line, 18, 20);
        second = readDouble(line, 21, 23);
    }
    const auto gpst = UTC2GPST(year, month, day, hour, minute, second);
    GlonassNavRecordEntry entry;
    entry.prn = prn;
    entry.record.year = year;
    entry.record.month = month;
    entry.record.day = day;
    entry.record.hour = hour;
    entry.record.minute = minute;
    entry.record.sec = second;
    entry.record.gpsWeek = gpst.week;
    entry.record.gpsSec = gpst.seconds;
    entry.record.a0 = readDouble(line, firstClockColumn, firstClockColumn + 18);
    entry.record.a1 = readDouble(line, firstClockColumn + 19, firstClockColumn + 37);
    return entry;
}

}  // namespace

NavHead ReadNavHead(std::istream& input) {
    NavHead head;
    std::string line;
    while (std::getline(input, line)) {
        const std::string keyword = field(line, 61, 80);
        if (keyword.find("RINEX VERSION / TYPE") != std::string::npos) {
            head.version = readInt(line, 6, 6);
        } else if (keyword.find("ION ALPHA") != std::string::npos) {
            for (int i = 0; i < 4; ++i) {
                const int start = 3 + i * 12;
                head.alpha[static_cast<std::size_t>(i)] = readDouble(line, start, start + 11);
            }
        } else if (keyword.find("ION BETA") != std::string::npos) {
            for (int i = 0; i < 4; ++i) {
                const int start = 3 + i * 12;
                head.beta[static_cast<std::size_t>(i)] = readDouble(line, start, start + 11);
            }
        } else if (keyword.find("LEAP SECONDS") != std::string::npos) {
            head.leapSeconds = readInt(line, 5, 6);
        } else if (keyword.find("END OF HEADER") != std::string::npos) {
            break;
        }
    }
    return head;
}

NavHead ReadNavHeadFile(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open navigation file");
    }
    return ReadNavHead(input);
}

std::vector<NavRecordEntry> ReadNavData(std::istream& input, char system, const NavHead& head) {
    return ReadNavData(input, system, head, RuntimeOptions{});
}

std::vector<NavRecordEntry> ReadNavData(std::istream& input, char system, const NavHead& head,
                                        const RuntimeOptions& options) {
    if (head.version < 2 || head.version > 4) {
        throw std::invalid_argument("unsupported RINEX navigation version");
    }

    NavHead mutableHead = head;
    const int maxPrn = Max_PRN_For_System(system);
    std::vector<std::vector<NavRecordEntry>> byPrn(static_cast<std::size_t>(maxPrn + 1));
    std::string line;
    while (std::getline(input, line)) {
        bool shouldReadBlock = false;
        bool IsCNAVBlock = false;
        if (head.version == 2) {
            shouldReadBlock = (system == 'G' || system == 'C' || system == 'E');
        } else if (head.version == 3) {
            shouldReadBlock = !readText(line, 1, 1).empty() && field(line, 1, 1)[0] == system;
        } else {
            if (field(line, 1, 7) == "> ION G") {
                Read_V4_Ion_Block(input, mutableHead);
                continue;
            }
            shouldReadBlock = Header_Matches(system, line, options.IsCNAV);
            IsCNAVBlock = options.IsCNAV && (system == 'G' || system == 'C' || system == 'J');
            if (shouldReadBlock && !Read_Line(input, line)) {
                break;
            }
        }
        if (!shouldReadBlock) {
            continue;
        }

        int year = 0;
        int month = 0;
        int day = 0;
        int hour = 0;
        int minute = 0;
        double second = 0.0;
        NavRecordEntry entry = Read_First_Nav_Line(line, system, head.version, year, month, day, hour, minute, second);
        if (entry.prn < 1 || entry.prn > maxPrn) {
            continue;
        }

        const bool ok = IsCNAVBlock
            ? Read_CNAV_Block(input, entry, head.version)
            : Read_Ordinary_Nav_Block(input, entry, head.version);
        if (!ok) {
            break;
        }

        const bool requireEccentricity = !(head.version == 4 && system == 'G' && !options.IsCNAV);
        if (!IsCNAVBlock && !Is_Usable_Record(entry.record, requireEccentricity)) {
            continue;
        }
        Keep_Epoch_Replacement(byPrn[static_cast<std::size_t>(entry.prn)], entry, head.version == 4);
    }

    std::vector<NavRecordEntry> records;
    for (const auto& group : byPrn) {
        records.insert(records.end(), group.begin(), group.end());
    }
    return records;
}

std::vector<NavRecordEntry> ReadNavDataFile(const std::string& path, char system, const NavHead& head) {
    return ReadNavDataFile(path, system, head, RuntimeOptions{});
}

std::vector<NavRecordEntry> ReadNavDataFile(const std::string& path, char system, const NavHead& head,
                                            const RuntimeOptions& options) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open navigation file");
    }
    ReadNavHead(input);
    return ReadNavData(input, system, head, options);
}

std::vector<GlonassNavRecordEntry> ReadNavData_R(std::istream& input, const NavHead& head) {
    if (head.version < 2 || head.version > 4) {
        throw std::invalid_argument("unsupported RINEX navigation version");
    }

    std::vector<GlonassNavRecordEntry> records;
    std::string line;
    while (std::getline(input, line)) {
        bool shouldReadBlock = false;
        if (head.version == 2) {
            shouldReadBlock = true;
        } else if (head.version == 3) {
            shouldReadBlock = !readText(line, 1, 1).empty() && field(line, 1, 1)[0] == 'R';
        } else {
            if (field(line, 1, 7) == "> ION G") {
                std::string ignored;
                Read_Line(input, ignored);
                Read_Line(input, ignored);
                Read_Line(input, ignored);
                continue;
            }
            shouldReadBlock = field(line, 1, 7) == "> EPH R";
            if (shouldReadBlock && !Read_Line(input, line)) {
                break;
            }
        }
        if (!shouldReadBlock) {
            continue;
        }

        GlonassNavRecordEntry entry = Read_First_Glonass_Line(line, head.version);
        if (entry.prn < 1 || entry.prn > constants::RNum) {
            continue;
        }

        const int firstColumn = head.version == 2 ? 4 : 5;
        if (!Read_Line(input, line)) break;
        entry.record.x = Read_Nav_Value(line, firstColumn, 0);
        entry.record.vx = Read_Nav_Value(line, firstColumn, 1);
        entry.record.ax = Read_Nav_Value(line, firstColumn, 2);
        entry.record.health = Read_Nav_Value(line, firstColumn, 3);

        if (!Read_Line(input, line)) break;
        entry.record.y = Read_Nav_Value(line, firstColumn, 0);
        entry.record.vy = Read_Nav_Value(line, firstColumn, 1);
        entry.record.ay = Read_Nav_Value(line, firstColumn, 2);
        entry.record.frequency = Read_Nav_Value(line, firstColumn, 3);

        if (!Read_Line(input, line)) break;
        entry.record.z = Read_Nav_Value(line, firstColumn, 0);
        entry.record.vz = Read_Nav_Value(line, firstColumn, 1);
        entry.record.az = Read_Nav_Value(line, firstColumn, 2);

        if (head.version == 4) {
            if (!Read_Line(input, line)) break;
            entry.record.tgd = Read_Nav_Value(line, firstColumn, 0);
        }
        records.push_back(entry);
    }
    return records;
}

std::vector<GlonassNavRecordEntry> ReadNavData_RFile(const std::string& path, const NavHead& head) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open navigation file");
    }
    ReadNavHead(input);
    return ReadNavData_R(input, head);
}

}  // namespace orbclkcmp
