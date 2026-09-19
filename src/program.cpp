#include <exception>
#include <iostream>
#include <string>

#include "main.hpp"

namespace {

void Print_Usage() {
    std::cout
        << "Usage: orbclkcmp_cpp [config_file]\n"
        << "\n"
        << "Orbit/clock comparison workflow.\n"
        << "\n"
        << "All processing options and product file names are read from a key=value config file.\n"
        << "If config_file is omitted, cmp.conf is used.\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            Print_Usage();
            return 0;
        }
    }

    try {
        if (argc > 2) {
            throw std::invalid_argument("usage: orbclkcmp_cpp [config_file]");
        }
        const std::string configPath = argc == 2 ? argv[1] : "cmp.conf";
        const orbclkcmp::Config config = orbclkcmp::Read_Config(configPath);
        return orbclkcmp::Run_Main(config, std::cout, std::cerr);
    } catch (const std::exception& error) {
        std::cerr << "orbclkcmp_cpp: " << error.what() << "\n";
        return 1;
    }
}
