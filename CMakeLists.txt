cmake_minimum_required(VERSION 3.50)
project(quad_tree VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_CLANG_TIDY clang-tidy)
set(CMAKE_CXX_CLANG_TIDY_ARGS "-checks=cppcoreguidelines-*,modernize-*")

add_subdirectory(src)

option(BUILD_MATH "Build the project's tests" ON)
if(BUILD_MATH)
  add_subdirectory(math)
endif()

option(BUILD_TESTING "Build the project's tests" ON)
if(BUILD_TESTING)
  add_subdirectory(benchmark)

  include(CTest)
  add_subdirectory(test)
endif()