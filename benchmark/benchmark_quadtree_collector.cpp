#include <benchmark/benchmark.h>

#include "quadtree/quadtree.h"
#include "quadtree/quadtree_collector.h"
#include "quadtree/quadtree_math.h"

// Define types as in test files
using QuadTreeType = qtree::QuadTree<int, qtree::vector3, qtree::bbox3, std::vector, std::allocator>;
using QuadTreeCollectorType = qtree::QuadTreeCollector<int, qtree::vector3, qtree::bbox3, std::vector, std::allocator>;

// Benchmark frustum culling collection with different tree depths
static void BM_CollectByFrustumDepth(benchmark::State& state) {
    const int depth = static_cast<int>(state.range(0));

    // Initialize quadtree
    QuadTreeType tree;
    qtree::vector3 min(-100.0f, -50.0f, -100.0f);
    qtree::vector3 max(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(min, max);
    tree.initialize(root_box, depth);

    // Set elements in multiple nodes
    size_t node_count = std::min(tree.get_number_nodes(), static_cast<size_t>(1000));
    for (size_t i = 0; i < node_count; ++i) {
        tree.get_node_by_index(i).set_element(static_cast<int>(i));
    }

    // Create a projection matrix
    qtree::matrix44 projection;
    projection.persp_fov_rh(45.0f, 1.0f, 0.1f, 1000.0f);
    projection.translate(qtree::vector3(0.0f, 0.0f, -50.0f));

    // Collector
    QuadTreeCollectorType collector;
    std::vector<int> collected;
    collected.reserve(1024);

    for ([[maybe_unused]] auto _ : state) {
        collector.collect_by_frustum(tree.get_root_node(), projection, collected);
        benchmark::DoNotOptimize(collected);
    }
}

// Benchmark line intersection collection with different tree depths
static void BM_CollectByLineIntersectDepth(benchmark::State& state) {
    const int depth = static_cast<int>(state.range(0));

    // Initialize quadtree
    QuadTreeType tree;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(center, extents);
    tree.initialize(root_box, depth);

    // Set elements in multiple nodes
    size_t node_count = std::min(tree.get_number_nodes(), static_cast<size_t>(1000));
    for (size_t i = 0; i < node_count; ++i) {
        tree.get_node_by_index(i).set_element(static_cast<int>(i));
    }

    // Create a line
    qtree::vector3 start(-50.0f, 0.0f, -50.0f);
    qtree::vector3 end(50.0f, 0.0f, 50.0f);
    qtree::line3 line(start, end);

    // Collector
    QuadTreeCollectorType collector;
    std::vector<int> collected;
    collected.reserve(1024);

    for ([[maybe_unused]] auto _ : state) {
        collector.collect_by_line_intersect(tree.get_root_node(), line, collected);
        benchmark::DoNotOptimize(collected);
    }
}

// Benchmark collector with varying number of elements
static void BM_CollectByFrustumElementCount(benchmark::State& state) {
    const int element_count = static_cast<int>(state.range(0));

    // Initialize quadtree
    QuadTreeType tree;
    qtree::vector3 min(-100.0f, -50.0f, -100.0f);
    qtree::vector3 max(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(min, max);
    tree.initialize(root_box, 5);

    // Set elements in nodes
    size_t node_count = std::min(tree.get_number_nodes(), static_cast<size_t>(element_count));
    for (size_t i = 0; i < node_count; ++i) {
        tree.get_node_by_index(i).set_element(static_cast<int>(i));
    }

    // Create a projection matrix
    qtree::matrix44 projection;
    projection.persp_fov_rh(45.0f, 1.0f, 0.1f, 1000.0f);
    projection.translate(qtree::vector3(0.0f, 0.0f, -50.0f));

    // Collector
    QuadTreeCollectorType collector;
    std::vector<int> collected;
    collected.reserve(1024);

    for ([[maybe_unused]] auto _ : state) {
        collector.collect_by_frustum(tree.get_root_node(), projection, collected);
        benchmark::DoNotOptimize(collected);
    }
}

BENCHMARK(BM_CollectByFrustumDepth)->DenseRange(1, 6);
BENCHMARK(BM_CollectByLineIntersectDepth)->DenseRange(1, 6);
BENCHMARK(BM_CollectByFrustumElementCount)->DenseRange(10, 1000, 100);

BENCHMARK_MAIN();