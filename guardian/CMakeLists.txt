# Copyright 2026 Ahmed Wafdy
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(guardian VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(GUARDIAN_BUILD_TESTS "Build host unit tests (GoogleTest)" OFF)
option(GUARDIAN_COVERAGE "Instrument for gcov coverage (host only)" OFF)

# NFR-Q-4: -Wall -Wextra -Werror on every target we own.
set(GUARDIAN_WARNINGS -Wall -Wextra -Werror -Wpedantic -Wshadow -Wconversion -Wsign-conversion)

# ---- RT core: platform-independent, no exceptions / RTTI (NFR-Q-1) ----
add_library(guardian_core STATIC
  src/guardian.cpp
  src/heartbeat_monitor.cpp
  src/state_machine.cpp
  src/validator.cpp
)
target_include_directories(guardian_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(guardian_core PRIVATE ${GUARDIAN_WARNINGS} -fno-exceptions -fno-rtti)

if(GUARDIAN_COVERAGE)
  target_compile_options(guardian_core PRIVATE --coverage -O0)
  target_link_options(guardian_core PUBLIC --coverage)
endif()

# ---- Executable: QNX timer+pulse loop, or POSIX clock_nanosleep loop ----
# Test the system name, not the QNX variable: CMake does not set QNX for this toolchain.
if(CMAKE_SYSTEM_NAME STREQUAL "QNX")
  add_executable(guardian src/platform/main_qnx.cpp)
  # Strict -std=c++17 hides the POSIX timer API and QNX extensions (SIGEV_PULSE_INIT).
  target_compile_definitions(guardian PRIVATE _QNX_SOURCE)
else()
  add_executable(guardian src/platform/main_posix.cpp)
endif()
target_link_libraries(guardian PRIVATE guardian_core)
target_compile_options(guardian PRIVATE ${GUARDIAN_WARNINGS} -fno-exceptions -fno-rtti)
install(TARGETS guardian RUNTIME DESTINATION bin)

# ---- Host unit tests ----
if(GUARDIAN_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    # Ubuntu ships GoogleTest sources at /usr/src/googletest (package `googletest`).
    set(GUARDIAN_GTEST_SRC "/usr/src/googletest" CACHE PATH "GoogleTest source dir")
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    add_subdirectory(${GUARDIAN_GTEST_SRC} ${CMAKE_BINARY_DIR}/googletest EXCLUDE_FROM_ALL)
    add_library(GTest::gtest_main ALIAS gtest_main)
  endif()
  include(GoogleTest)

  add_executable(guardian_tests
    test/test_state_machine.cpp
    test/test_validator.cpp
    test/test_heartbeat_monitor.cpp
    test/test_event_log.cpp
    test/test_guardian.cpp
  )
  target_link_libraries(guardian_tests PRIVATE guardian_core GTest::gtest_main)
  target_compile_options(guardian_tests PRIVATE -Wall -Wextra -Werror)
  gtest_discover_tests(guardian_tests)
endif()
