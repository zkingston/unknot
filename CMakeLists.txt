cmake_minimum_required(VERSION 3.16...3.22)

project(
  knot
  VERSION 0.1
  LANGUAGES C CXX
)

if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif ()

option(UNKNOT_VIZ "Build visualization components" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(FetchInitCPM)
include(Dependencies)

add_executable(unknot src/unknot.cc)
target_link_libraries(unknot PRIVATE
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  cxxopts::cxxopts
  fmt::fmt
)

if(UNKNOT_VIZ)
  target_link_libraries(unknot PRIVATE
    implot3d implot
  )
  target_compile_options(unknot PRIVATE -DUNKNOT_VIZ)
endif()

target_compile_options(unknot PRIVATE -O3 -march=native -Wall -Wextra -Wpedantic)
