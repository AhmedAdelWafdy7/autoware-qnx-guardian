# Copyright 2026 Ahmed Wafdy
# SPDX-License-Identifier: Apache-2.0
#
# Host-only oracle test: Guardian RSS formulas vs intel/ad-rss-lib. Built by run.sh, which
# fetches and installs ad-rss-lib into CMAKE_PREFIX_PATH. Not part of the Guardian build.
cmake_minimum_required(VERSION 3.16)
project(guardian_rss_oracle LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(GUARDIAN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH "guardian/ source directory")

find_package(ad_rss CONFIG REQUIRED)

find_package(GTest QUIET)
if(NOT GTest_FOUND)
  set(GUARDIAN_GTEST_SRC "/usr/src/googletest" CACHE PATH "GoogleTest source dir")
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  add_subdirectory(${GUARDIAN_GTEST_SRC} ${CMAKE_BINARY_DIR}/googletest EXCLUDE_FROM_ALL)
  add_library(GTest::gtest_main ALIAS gtest_main)
endif()

add_executable(rss_oracle_test
  rss_oracle_test.cpp
  ${GUARDIAN_DIR}/rss/src/rss.cpp
)
target_include_directories(rss_oracle_test PRIVATE
  ${GUARDIAN_DIR}/include
  ${GUARDIAN_DIR}/rss/include
)
target_link_libraries(rss_oracle_test PRIVATE ad_rss GTest::gtest_main)
target_compile_options(rss_oracle_test PRIVATE -Wall -Wextra)
