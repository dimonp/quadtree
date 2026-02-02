find_package(Eigen3 CONFIG REQUIRED)

add_library(quadtree_math INTERFACE)

target_sources(quadtree_math
    PUBLIC FILE_SET HEADERS
    BASE_DIRS ${PROJECT_SOURCE_DIR}/include
    FILES
        ${PROJECT_SOURCE_DIR}/include/quadtree/quadtree_math.h)

target_include_directories(quadtree_math INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/quadtree>
)

target_link_libraries(quadtree_math INTERFACE quadtree_project_options)
target_link_libraries(quadtree_math INTERFACE Eigen3::Eigen)

install(TARGETS quadtree_math
    EXPORT quadtree_math_targets
    FILE_SET HEADERS DESTINATION include)

install(EXPORT quadtree_math_targets
        FILE quadtree_math_targets.cmake
        NAMESPACE quadtree::
        DESTINATION cmake)