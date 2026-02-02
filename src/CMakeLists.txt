find_package(Microsoft.GSL CONFIG REQUIRED)

add_library(quadtree INTERFACE)
add_library(quadtree::quadtree ALIAS quadtree)

target_sources(quadtree
    PUBLIC FILE_SET HEADERS
    BASE_DIRS ${PROJECT_SOURCE_DIR}/include
    FILES
        ${PROJECT_SOURCE_DIR}/include/quadtree/quadtree.h
        ${PROJECT_SOURCE_DIR}/include/quadtree/quadtree_collector.h)

target_include_directories(quadtree INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/quadtree>)

target_link_libraries(quadtree INTERFACE quadtree_project_options)
target_link_libraries(quadtree INTERFACE Microsoft.GSL::GSL)

install(TARGETS quadtree
    EXPORT quadtree_targets
    FILE_SET HEADERS DESTINATION include)

install(EXPORT quadtree_targets
        FILE quadtree_targets.cmake
        NAMESPACE quadtree::
        DESTINATION cmake)