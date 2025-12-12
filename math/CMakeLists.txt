include(FetchContent)
FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 5.0.1
  # Other content options...
  EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(Eigen)

add_library(quadtree_math
    quadtree_math.h
)
target_include_directories(quadtree_math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(quadtree_math PUBLIC Eigen3::Eigen)
set_target_properties(quadtree_math PROPERTIES LINKER_LANGUAGE CXX)


