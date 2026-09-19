#include "read_sp3.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "standard_reader.hpp"
#include "time_convert.hpp"

namespace orbclkcmp {

namespace {

void Shift_SP3Data(Sp3Data& data) {
    for (int i = 0; i < constants::Sp3EpochSlots - 1; ++i) {
        data.gpsWeek[i] = data.gpsWeek[i + 1];
        data.gpsSec[i] = data.gpsSec[i + 1];
    }
    data.gpsWeek[constants::Sp3EpochSlots - 1] = 0;
    data.gpsSec[constants::Sp3EpochSlots - 1] = 0.0;

    for (auto& eph : data.eph) {
        for (int component = 0; component < 3; ++component) {
            for (int slot = 0; slot < constants::Sp3EpochSlots - 1; ++slot) {
                eph.coor[component][slot] = eph.coor[component][slot + 1];
            }
            eph.coor[component][constants::Sp3EpochSlots - 1] = 9999.0;
        }
        for (int slot = 0; slot < constants::Sp3EpochSlots - 1; ++slot) {
            eph.clk[slot] = eph.clk[slot + 1];
        }
        eph.clk[constants::Sp3EpochSlots - 1] = 999999.999999;
    }
}

bool Is_Epoch_Line(const std::string& line) {
    return !line.empty() && line[0] == '*';
}

std::string Right_Trim(std::string value) {
    while (!value.empty() && value.back() == ' ') {
        value.pop_back();
    }
    return value;
}

GpstTime Read_SP3_Epoch_Time(const std::string& line) {
    if (!Is_Epoch_Line(line)) {
        throw std::invalid_argument("SP3 epoch line must start with '*'");
    }

    char marker = '\0';
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;

    std::istringstream stream(line);
    stream >> marker >> year >> month >> day >> hour >> minute >> second;
    if (!stream || marker != '*') {
        throw std::invalid_argument("invalid SP3 epoch line: " + line);
    }

    return UTC2GPST(year, month, day, hour, minute, second);
}

}  // namespace

Sp3Head ReadSP3Head(std::istream& input, std::string* firstEpochLine) {
    Sp3Head head;

    std::string line;
    if (!std::getline(input, line)) {
        throw std::runtime_error("SP3 file is empty");
    }

    if (!std::getline(input, line)) {
        throw std::runtime_error("SP3 file missing second header line");
    }
    head.gpsWeek = readInt(line, 4, 7);
    head.gpsSec = readDouble(line, 8, 22);

    if (!std::getline(input, line)) {
        throw std::runtime_error("SP3 file missing satellite list line");
    }
    head.prnCount = readInt(line, 4, 6);
    std::string satelliteList = Right_Trim(field(line, 10, 60));

    int listLineCount = 1;
    while (listLineCount * 17 < head.prnCount) {
        if (!std::getline(input, line)) {
            throw std::runtime_error("SP3 satellite list ended unexpectedly");
        }
        satelliteList += Right_Trim(field(line, 10, 60));
        ++listLineCount;
    }

    head.system.resize(static_cast<std::size_t>(head.prnCount));
    head.prn.resize(static_cast<std::size_t>(head.prnCount));
    for (int i = 0; i < head.prnCount; ++i) {
        const int start = i * 3 + 1;
        const std::string token = std::string(field(satelliteList, start, start + 2));
        head.system[static_cast<std::size_t>(i)] = token[0];
        head.prn[static_cast<std::size_t>(i)] = readInt(token, 2, 3);
    }

    std::string accuracyList;
    while (std::getline(input, line)) {
        if (line.find("  cc ") != std::string::npos) {
            const std::string timeSystem = readText(line, 10, 12);
            if (timeSystem == "GPS") {
                head.sp3Time = 0.0;
            } else if (timeSystem == "BDS" || timeSystem == "BDT") {
                head.sp3Time = -14.0;
            } else {
                throw std::runtime_error("unknown SP3 time system: " + timeSystem);
            }
        }

        if (line.find("++") != std::string::npos) {
            accuracyList += Right_Trim(field(line, 10, static_cast<int>(line.size())));
        } else if (Is_Epoch_Line(line)) {
            if (firstEpochLine != nullptr) {
                *firstEpochLine = line;
            }
            break;
        }
    }

    for (int i = 0; i < head.prnCount; ++i) {
        const int index = Get_PRNIndex(head.system[static_cast<std::size_t>(i)], head.prn[static_cast<std::size_t>(i)]);
        if (index < 0 || index >= static_cast<int>(head.orbitAccuracy.size())) {
            continue;
        }

        const int start = i * 3 + 1;
        if (start + 2 <= static_cast<int>(accuracyList.size())) {
            const std::string accuracy = trim(field(accuracyList, start, start + 2));
            // SP3 uses *** when the header accuracy exponent is unavailable.
            int accuracyValue = 0;
            if (!accuracy.empty() && accuracy.find('*') == std::string::npos) {
                accuracyValue = readInt(accuracy, 1, static_cast<int>(accuracy.size()));
            }
            head.orbitAccuracy.at(static_cast<std::size_t>(index)) = accuracyValue;
        }
    }

    return head;
}

Sp3Head ReadSP3HeadFile(const std::string& path, std::string* firstEpochLine) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open SP3 file: " + path);
    }
    return ReadSP3Head(input, firstEpochLine);
}

int ReadSP3Data(
    std::istream& input,
    const Sp3Head& head,
    Sp3Data& data,
    int epochCount,
    std::string firstEpochLine) {
    int epochsRead = 0;
    std::string line = std::move(firstEpochLine);

    while (epochsRead < epochCount) {
        if (line.empty() && !std::getline(input, line)) {
            break;
        }

        if (!Is_Epoch_Line(line)) {
            line.clear();
            continue;
        }

        Shift_SP3Data(data);
        const auto epochTime = Read_SP3_Epoch_Time(line);
        data.gpsWeek[constants::Sp3EpochSlots - 1] = epochTime.week;
        data.gpsSec[constants::Sp3EpochSlots - 1] = epochTime.seconds;

        int recordsSeen = 0;
        while (recordsSeen < head.prnCount && std::getline(input, line)) {
            if (Is_Epoch_Line(line)) {
                break;
            }

            const std::string symbol = readText(line, 1, 1);
            if (symbol != "P") {
                continue;
            }

            const char system = readText(line, 2, 2)[0];
            const int prn = readInt(line, 3, 4);
            const int index = Get_PRNIndex(system, prn);
            ++recordsSeen;
            if (index < 0) {
                continue;
            }

            data.eph[static_cast<std::size_t>(index)].coor[0][constants::Sp3EpochSlots - 1] = readDouble(line, 5, 18);
            data.eph[static_cast<std::size_t>(index)].coor[1][constants::Sp3EpochSlots - 1] = readDouble(line, 19, 32);
            data.eph[static_cast<std::size_t>(index)].coor[2][constants::Sp3EpochSlots - 1] = readDouble(line, 33, 46);
            data.eph[static_cast<std::size_t>(index)].clk[constants::Sp3EpochSlots - 1] = readDouble(line, 47, 60);
        }

        ++epochsRead;
        if (!Is_Epoch_Line(line)) {
            line.clear();
        }
    }

    return epochsRead;
}

int ReadSP3Data(
    std::istream& input,
    const Sp3Head& head,
    Sp3Data& data,
    int epochCount,
    std::string* pendingEpochLine) {
    int epochsRead = 0;
    std::string line;
    if (pendingEpochLine != nullptr) {
        line = std::move(*pendingEpochLine);
        pendingEpochLine->clear();
    }

    while (epochsRead < epochCount) {
        if (line.empty() && !std::getline(input, line)) {
            break;
        }

        if (!Is_Epoch_Line(line)) {
            line.clear();
            continue;
        }

        Shift_SP3Data(data);
        const auto epochTime = Read_SP3_Epoch_Time(line);
        data.gpsWeek[constants::Sp3EpochSlots - 1] = epochTime.week;
        data.gpsSec[constants::Sp3EpochSlots - 1] = epochTime.seconds;

        int recordsSeen = 0;
        while (recordsSeen < head.prnCount && std::getline(input, line)) {
            if (Is_Epoch_Line(line)) {
                break;
            }

            const std::string symbol = readText(line, 1, 1);
            if (symbol != "P") {
                continue;
            }

            const char system = readText(line, 2, 2)[0];
            const int prn = readInt(line, 3, 4);
            const int index = Get_PRNIndex(system, prn);
            ++recordsSeen;
            if (index < 0) {
                continue;
            }

            data.eph[static_cast<std::size_t>(index)].coor[0][constants::Sp3EpochSlots - 1] = readDouble(line, 5, 18);
            data.eph[static_cast<std::size_t>(index)].coor[1][constants::Sp3EpochSlots - 1] = readDouble(line, 19, 32);
            data.eph[static_cast<std::size_t>(index)].coor[2][constants::Sp3EpochSlots - 1] = readDouble(line, 33, 46);
            data.eph[static_cast<std::size_t>(index)].clk[constants::Sp3EpochSlots - 1] = readDouble(line, 47, 60);
        }

        ++epochsRead;
        if (Is_Epoch_Line(line)) {
            if (pendingEpochLine != nullptr) {
                *pendingEpochLine = line;
            }
        } else {
            line.clear();
        }
    }

    return epochsRead;
}

int ReadSP3DataFile(const std::string& path, const Sp3Head& head, Sp3Data& data, int epochCount) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open SP3 file: " + path);
    }

    std::string firstEpochLine;
    ReadSP3Head(input, &firstEpochLine);
    return ReadSP3Data(input, head, data, epochCount, firstEpochLine);
}

}  // namespace orbclkcmp
