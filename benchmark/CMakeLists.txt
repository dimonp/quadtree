find_package(benchmark CONFIG REQUIRED)

add_executable(quadtree_benchmark benchmark_quadtree.cpp)
target_include_directories(quadtree_benchmark PRIVATE quadtree)
target_include_directories(quadtree_benchmark PRIVATE quadtree_math)
target_link_libraries(quadtree_benchmark PRIVATE
    benchmark::benchmark
    quadtree
    quadtree_math
)

add_executable(quadtree_collector_benchmark benchmark_quadtree_collector.cpp)
target_include_directories(quadtree_collector_benchmark PRIVATE quadtree)
target_include_directories(quadtree_collector_benchmark PRIVATE quadtree_math)
target_link_libraries(quadtree_collector_benchmark PRIVATE
    benchmark::benchmark
    quadtree
    quadtree_math
)
