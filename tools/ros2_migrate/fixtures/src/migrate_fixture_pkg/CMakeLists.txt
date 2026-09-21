cmake_minimum_required(VERSION 3.16)
project(migrate_fixture_pkg)

# The matrix needs every fixture TU in compile_commands.json; a single shared
# library over all sources gives one entry per file.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

file(GLOB FIXTURE_SOURCES CONFIGURE_DEPENDS src/*.cpp)
# safe_macro_from_flag.cpp gets its colliding macro from the
# COMPILE COMMAND (a -D in its compile_commands.json entry), per-source so
# no other fixture sees it.
set_source_files_properties(src/safe_macro_from_flag.cpp PROPERTIES
  COMPILE_DEFINITIONS "loaned=42")
add_library(migrate_fixtures SHARED ${FIXTURE_SOURCES})
ament_target_dependencies(migrate_fixtures rclcpp std_msgs geometry_msgs)
# SUPPORT SURFACE: this fixture workspace builds inside the ros2-bench Linux
# container only (the matrix's home — see ../../README.md); the MSVC
# properties below are the one-line, no-op-on-Linux courtesies for anyone
# porting the matrix, not a tested platform claim.
set_target_properties(migrate_fixtures PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

# The behavior pin: publishes through safe_unique before/after migration.
add_executable(matrix_runner matrix_runner.cpp)
target_link_libraries(matrix_runner migrate_fixtures)
ament_target_dependencies(matrix_runner rclcpp std_msgs geometry_msgs)

# Install BOTH: the installed runner links the shared fixture library, so
# `ros2 run migrate_fixture_pkg matrix_runner` must find it post-install
# (the matrix itself runs from build/, where the build-tree RPATH covers it).
install(TARGETS migrate_fixtures
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib)
install(TARGETS matrix_runner DESTINATION lib/${PROJECT_NAME})

ament_package()
