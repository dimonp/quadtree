find_package(Microsoft.GSL CONFIG REQUIRED)

add_library(quadtree INTERFACE
    quadtree.h
    quadtree_collector.h
)

target_include_directories(quadtree INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/quadtree>
)
target_link_libraries(quadtree INTERFACE Microsoft.GSL::GSL)

install(TARGETS quadtree
    EXPORT quadtree_targets
    INCLUDES DESTINATION include)

install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/quadtree.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/quadtree_collector.h"
    DESTINATION include/quadtree)

install(EXPORT quadtree_targets
        FILE quadtree_targets.cmake
        NAMESPACE quadtree::
        DESTINATION cmake)