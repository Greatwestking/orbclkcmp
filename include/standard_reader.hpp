#pragma once

#include <string>
#include <string_view>

namespace orbclkcmp {

// Read a fixed-width field by 1-based inclusive columns.
std::string field(std::string_view line, int start, int end);

std::string trim(std::string_view value);

// Read an integer from fixed-width columns.
int readInt(std::string_view line, int start, int end);

// Read a real number from fixed-width columns.
double readDouble(std::string_view line, int start, int end);

// Read text from fixed-width columns.
std::string readText(std::string_view line, int start, int end);

}  // namespace orbclkcmp
