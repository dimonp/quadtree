cmake_minimum_required(VERSION 3.50)

option(QUADTREE_BUILD_TESTING "Build the project's tests" ON)
option(QUADTREE_ENABLE_SANITIZERS "Enable all sanitizers" ON)
option(QUADTREE_BUILD_MATH "Build the project's tests" ON)
option(QUADTREE_ENABLE_EXCEPTIONS "Enable exceptions" OFF)

if(QUADTREE_BUILD_MATH)
  list(APPEND VCPKG_MANIFEST_FEATURES "math")
endif()

if(QUADTREE_BUILD_TESTING)
  list(APPEND VCPKG_MANIFEST_FEATURES "tests")
endif()

if (NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED True) 
endif()

project(quad_tree VERSION 0.1 LANGUAGES CXX)

add_library(quadtree_project_options INTERFACE IMPORTED)

if (QUADTREE_ENABLE_SANITIZERS)
  if(MSVC)
    target_compile_options(quadtree_project_options INTERFACE /fsanitize=address /Zi /INCREMENTAL:NO)
    target_link_options(quadtree_project_options INTERFACE /INCREMENTAL:NO)
  else()
    target_compile_options(quadtree_project_options INTERFACE -fsanitize=address,leak,undefined)
    target_link_options(quadtree_project_options INTERFACE -fsanitize=address,leak,undefined)
  endif()
endif()

if (QUADTREE_ENABLE_EXCEPTIONS)
  if(MSVC)
    target_compile_options(quadtree_project_options INTERFACE /EHsc-)
    target_compile_options(quadtree_project_options INTERFACE /EHs-)
    target_compile_definitions(quadtree_project_options INTERFACE _HAS_EXCEPTIONS=0)  
  else()
    target_compile_options(quadtree_project_options INTERFACE -fno-exceptions)
  endif()
endif()

find_program(CLANGTIDY clang-tidy)
if(CLANGTIDY)
  set(CMAKE_CXX_CLANG_TIDY "clang-tidy")
  set(CMAKE_CXX_CLANG_TIDY_ARGS "-checks=cppcoreguidelines-*,modernize-*")
endif()

add_subdirectory(src)

if(QUADTREE_BUILD_MATH)
  add_subdirectory(math)
endif()

if(QUADTREE_BUILD_TESTING AND QUADTREE_BUILD_MATH)
  add_subdirectory(benchmark)
  include(CTest)
  add_subdirectory(test)
endif()