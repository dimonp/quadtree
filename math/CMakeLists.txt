find_package(Eigen3 CONFIG REQUIRED)

add_library(quadtree_math INTERFACE)

target_include_directories(quadtree_math INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(quadtree_math INTERFACE quadtree_project_options)
target_link_libraries(quadtree_math INTERFACE Eigen3::Eigen)

install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/quadtree_math.h"
    DESTINATION include/quadtree)

