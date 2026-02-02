find_package(benchmark CONFIG REQUIRED)

add_executable(quadtree_benchmark benchmark_quadtree.cpp)
target_link_libraries(quadtree_benchmark PRIVATE quadtree_project_options)
target_link_libraries(quadtree_benchmark PRIVATE
    benchmark::benchmark
    quadtree
    quadtree_math
)

add_executable(quadtree_collector_benchmark benchmark_quadtree_collector.cpp)
target_link_libraries(quadtree_collector_benchmark PRIVATE quadtree_project_options)
target_link_libraries(quadtree_collector_benchmark PRIVATE
    benchmark::benchmark
    quadtree
    quadtree_math
)
