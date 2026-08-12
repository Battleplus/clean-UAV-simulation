cmake_minimum_required(VERSION 3.16)
project(drone_motor_system)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GZ_SIM REQUIRED IMPORTED_TARGET gz-sim8)

add_library(drone_latest_wrench_system SHARED
  src/LatestWrenchSystem.cc
)
target_compile_features(drone_latest_wrench_system PUBLIC cxx_std_17)
target_link_libraries(drone_latest_wrench_system PRIVATE PkgConfig::GZ_SIM)

install(TARGETS drone_latest_wrench_system
  LIBRARY DESTINATION lib
)

ament_package()
