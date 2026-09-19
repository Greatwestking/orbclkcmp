cmake_minimum_required(VERSION 3.16)

project(orbclkcmp_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(orbclkcmp_core
    src/cal_sat_clk.cpp
    src/cal_sat_pos_nav.cpp
    src/cal_sat_pos_sp3.cpp
    src/cross.cpp
    src/get_ant.cpp
    src/lagrange.cpp
    src/main.cpp
    src/read_clk.cpp
    src/read_mgex_dcb.cpp
    src/read_nav.cpp
    src/read_sp3.cpp
    src/prn_index.cpp
    src/satellite_pco.cpp
    src/ssr.cpp
    src/standard_reader.cpp
    src/time_convert.cpp
    src/types.cpp
    src/xyz_coordinate.cpp
)

target_include_directories(orbclkcmp_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(orbclkcmp_cpp
    src/program.cpp
)

set_target_properties(orbclkcmp_cpp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_CURRENT_SOURCE_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(orbclkcmp_cpp
    PRIVATE
        orbclkcmp_core
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/smoke_test.cpp")
    enable_testing()

    add_executable(orbclkcmp_tests
        tests/smoke_test.cpp
    )

    target_link_libraries(orbclkcmp_tests
        PRIVATE
            orbclkcmp_core
    )

    add_test(NAME smoke COMMAND orbclkcmp_tests)
endif()
