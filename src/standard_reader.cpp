#include "standard_reader.hpp"

#include <charconv>
#include <cctype>
#include <cstdlib>
#include <stdexcept>

namespace orbclkcmp {

std::string field(std::string_view line, int start, int end) {
    if (start < 1 || end < start) {
        throw std::out_of_range("invalid fixed-width field range");
    }

    const auto requestedLength = static_cast<std::size_t>(end - start + 1);
    std::string result(requestedLength, ' ');

    const auto zeroBasedStart = static_cast<std::size_t>(start - 1);
    if (zeroBasedStart >= line.size()) {
        return result;
    }

    const auto availableLength = line.size() - zeroBasedStart;
    const auto length = requestedLength < availableLength ? requestedLength : availableLength;
    result.replace(0, length, line.substr(zeroBasedStart, length));
    return result;
}

std::string trim(std::string_view value) {
    std::size_t begin = 0;
    while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin]))) {
        ++begin;
    }

    std::size_t end = value.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1]))) {
        --end;
    }

    return std::string(value.substr(begin, end - begin));
}

int readInt(std::string_view line, int start, int end) {
    const std::string text = trim(field(line, start, end));
    if (text.empty()) {
        throw std::invalid_argument("empty integer field");
    }

    int value = 0;
    const auto* first = text.data();
    const auto* last = text.data() + text.size();
    const auto result = std::from_chars(first, last, value);
    if (result.ec != std::errc{} || result.ptr != last) {
        throw std::invalid_argument("invalid integer field: " + text);
    }
    return value;
}

double readDouble(std::string_view line, int start, int end) {
    std::string text = trim(field(line, start, end));
    if (text.empty()) {
        return 0.0;
    }

    for (char& ch : text) {
        if (ch == 'D' || ch == 'd') {
            ch = 'e';
        }
    }

    char* endPtr = nullptr;
    const double value = std::strtod(text.c_str(), &endPtr);
    if (endPtr == text.c_str() || *endPtr != '\0') {
        throw std::invalid_argument("invalid floating-point field: " + text);
    }
    return value;
}

std::string readText(std::string_view line, int start, int end) {
    return trim(field(line, start, end));
}

}  // namespace orbclkcmp
