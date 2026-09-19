#include "read_clk.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "prn_index.hpp"
#include "time_convert.hpp"

namespace orbclkcmp {

namespace {

struct ClockLine {
    std::string type;
    std::string name;
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    int valueCount = 0;
    double clock = 0.0;
};

bool Read_Clock_Line(const std::string& line, ClockLine& record, bool requireClockValue) {
    std::istringstream stream(line);
    stream >> record.type >> record.name >> record.year >> record.month >> record.day >>
        record.hour >> record.minute >> record.second;
    if (!stream) {
        return false;
    }

    if (requireClockValue) {
        stream >> record.valueCount >> record.clock;
        return static_cast<bool>(stream);
    }
    return true;
}

void Shift_Clock_Data(ClockData& data) {
    for (auto& record : data.as) {
        if (record.clk[0] != 9999.0 && record.clk[1] != 9999.0) {
            const double dt =
                (data.gpsWeek[1] - data.gpsWeek[0]) * 604800.0 +
                data.gpsSec[1] - data.gpsSec[0];
            if (dt != 0.0) {
                record.clkVel = (record.clk[1] - record.clk[0]) / dt;
            }
        }
    }

    data.gpsWeek[0] = data.gpsWeek[1];
    data.gpsSec[0] = data.gpsSec[1];
    for (auto& record : data.as) {
        record.clk[0] = record.clk[1];
        record.clk[1] = 9999.0;
    }
}

}  // namespace

void ReadClkHead(std::istream& input) {
    std::string line;
    while (std::getline(input, line)) {
        if (line.find("END OF HEADER") != std::string::npos) {
            return;
        }
    }
}

int ReadClkEpoch(std::istream& input, ClockData& data, std::string* pendingLine) {
    std::string line;
    if (pendingLine != nullptr && !pendingLine->empty()) {
        line = std::move(*pendingLine);
        pendingLine->clear();
    } else if (!std::getline(input, line)) {
        return 0;
    }

    ClockLine firstRecord;
    if (!Read_Clock_Line(line, firstRecord, false)) {
        return 0;
    }

    Shift_Clock_Data(data);
    const auto epochTime = UTC2GPST(
        firstRecord.year,
        firstRecord.month,
        firstRecord.day,
        firstRecord.hour,
        firstRecord.minute,
        firstRecord.second);
    data.gpsWeek[1] = epochTime.week;
    data.gpsSec[1] = epochTime.seconds;

    while (true) {
        ClockLine record;
        if (!Read_Clock_Line(line, record, true)) {
            return 1;
        }

        if (record.minute != firstRecord.minute || std::abs(record.second - firstRecord.second) > 0.1) {
            if (pendingLine != nullptr) {
                *pendingLine = line;
            }
            return 1;
        }

        if (record.type == "AS" && record.name.size() >= 3) {
            const char system = record.name[0];
            const int prn = std::stoi(record.name.substr(1, 2));
            const int index = Get_PRNIndex(system, prn);
            if (index >= 0) {
                data.as[static_cast<std::size_t>(index)].clk[1] = record.clock;
            }
        }

        if (!std::getline(input, line)) {
            return 1;
        }
    }
}

int ReadClkData(std::istream& input, ClockData& data, int epochCount, std::string* pendingLine) {
    int epochsRead = 0;
    std::string localPendingLine;
    std::string* state = pendingLine != nullptr ? pendingLine : &localPendingLine;
    while (epochsRead < epochCount) {
        const int readCount = ReadClkEpoch(input, data, state);
        if (readCount == 0) {
            break;
        }
        epochsRead += readCount;
    }
    return epochsRead;
}

void ReadClkHeadFile(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open CLK file: " + path);
    }
    ReadClkHead(input);
}

int ReadClkDataFile(const std::string& path, ClockData& data, int epochCount) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open CLK file: " + path);
    }
    ReadClkHead(input);
    return ReadClkData(input, data, epochCount);
}

}  // namespace orbclkcmp
