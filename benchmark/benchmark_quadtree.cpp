#include <random>
#include <benchmark/benchmark.h>

#include "quadtree.h"
#include "quadtree_math.h"

// Define types for our quadtree
using QuadTreeType = qtree::QuadTree<int, qtree::vector3, qtree::bbox3, std::vector, std::allocator>;
using NodeType = qtree::QuadTree<int, qtree::vector3, qtree::bbox3, std::vector, std::allocator>::Node<int>;

// Benchmark fixture for QuadTree operations
class QuadTreeBenchmark : public benchmark::Fixture {
public:
    QuadTreeType tree;
    qtree::bbox3 root_box;
    std::vector<qtree::bbox3> test_boxes;
    std::mt19937 rng;
    std::uniform_real_distribution<float> dist;

    void SetUp(const ::benchmark::State& state) {
        // Initialize random number generator
        rng.seed(12345);
        dist = std::uniform_real_distribution<float>(-90.0f, 90.0f);

        // Initialize quadtree
        qtree::vector3 center(0.0f, 0.0f, 0.0f);
        qtree::vector3 extents(100.0f, 50.0f, 100.0f);
        root_box = qtree::bbox3(center, extents);
        tree.initialize(root_box, 5); // Use depth 5 for reasonable number of nodes

        // Generate test boxes for find_containment_node operations
        test_boxes.reserve(1000);
        for (int i = 0; i < 1000; ++i) {
            qtree::vector3 box_center(dist(rng), 0.0f, dist(rng));
            qtree::vector3 box_extents(5.0f, 5.0f, 5.0f);
            test_boxes.emplace_back(box_center, box_extents);
        }
    }
};

// Benchmark initialization
static void BM_QuadTreeInitialize(benchmark::State& state) {
    qtree::bbox3 root_box;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    root_box = qtree::bbox3(center, extents);

    for ([[maybe_unused]] auto _ : state) {
        QuadTreeType tree;
        tree.initialize(root_box, 4);
        benchmark::DoNotOptimize(tree);
    }
}

// Benchmark get_root_bbox
static void BM_QuadTreeGetRootBbox(benchmark::State& state) {
    QuadTreeType tree;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(center, extents);
    tree.initialize(root_box, 5);

    for ([[maybe_unused]] auto _ : state) {
        qtree::bbox3 bbox = tree.get_root_bbox();
        benchmark::DoNotOptimize(bbox);
    }
}

// Benchmark get_tree_depth
static void BM_QuadTreeGetTreeDepth(benchmark::State& state) {
    QuadTreeType tree;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(center, extents);
    tree.initialize(root_box, 5);

    for ([[maybe_unused]] auto _ : state) {
        uint8_t depth = tree.get_tree_depth();
        benchmark::DoNotOptimize(depth);
    }
}

// Benchmark calculate_number_nodes
static void BM_QuadTreeCalculateNumberNodes(benchmark::State& state) {
    QuadTreeType tree;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(center, extents);
    tree.initialize(root_box, 5);

    for ([[maybe_unused]] auto _ : state) {
        size_t count = tree.calculate_number_nodes(3);
        benchmark::DoNotOptimize(count);
    }
}

// Benchmark get_number_nodes
static void BM_QuadTreeGetNumberNodes(benchmark::State& state) {
    QuadTreeType tree;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(center, extents);
    tree.initialize(root_box, 5);

    for ([[maybe_unused]] auto _ : state) {
        size_t count = tree.get_number_nodes();
        benchmark::DoNotOptimize(count);
    }
}

// Benchmark get_root_node
static void BM_QuadTreeGetRootNode(benchmark::State& state) {
    QuadTreeType tree;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(center, extents);
    tree.initialize(root_box, 5);

    for ([[maybe_unused]] auto _ : state) {
        NodeType node = tree.get_root_node();
        benchmark::DoNotOptimize(node);
    }
}

// Benchmark find_containment_node_recursive
static void BM_QuadTreeFindContainmentNode(benchmark::State& state) {
    QuadTreeType tree;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(center, extents);
    tree.initialize(root_box, 5);

    // Generate test boxes
    std::vector<qtree::bbox3> test_boxes;
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> dist(-90.0f, 90.0f);

    test_boxes.reserve(1000);
    for (int i = 0; i < 1000; ++i) {
        qtree::vector3 box_center(dist(rng), 0.0f, dist(rng));
        qtree::vector3 box_extents(5.0f, 5.0f, 5.0f);
        test_boxes.emplace_back(box_center, box_extents);
    }

    size_t index = 0;
    for ([[maybe_unused]] auto _ : state) {
        const NodeType* node = tree.find_containment_node(test_boxes[index]);
        benchmark::DoNotOptimize(node);
        index = (index + 1) % test_boxes.size();
    }
}

// Benchmark Node operations
static void BM_NodeGetElement(benchmark::State& state) {
    QuadTreeType tree;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(center, extents);
    tree.initialize(root_box, 5);

    NodeType& root_node = tree.get_root_node();
    root_node.set_element(42);

    for ([[maybe_unused]] auto _ : state) {
        int element = root_node.get_element();
        benchmark::DoNotOptimize(element);
    }
}

static void BM_NodeSetElement(benchmark::State& state) {
    QuadTreeType tree;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(center, extents);
    tree.initialize(root_box, 5);

    NodeType& root_node = tree.get_root_node();
    int value = 42;

    for ([[maybe_unused]] auto _ : state) {
        root_node.set_element(value);
        benchmark::DoNotOptimize(value);
        value++;
    }
}

static void BM_NodeGetBbox(benchmark::State& state) {
    QuadTreeType tree;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(center, extents);
    tree.initialize(root_box, 5);

    const NodeType& root_node = tree.get_root_node();

    for ([[maybe_unused]] auto _ : state) {
        qtree::bbox3 bbox = root_node.get_bbox();
        benchmark::DoNotOptimize(bbox);
    }
}

static void BM_NodeGetChildAt(benchmark::State& state) {
    QuadTreeType tree;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(center, extents);
    tree.initialize(root_box, 5);

    const NodeType& root_node = tree.get_root_node();

    for ([[maybe_unused]] auto _ : state) {
        const NodeType* child = root_node.get_child_at(0);
        benchmark::DoNotOptimize(child);
    }
}

static void BM_NodeHasChildren(benchmark::State& state) {
    QuadTreeType tree;
    qtree::vector3 center(0.0f, 0.0f, 0.0f);
    qtree::vector3 extents(100.0f, 50.0f, 100.0f);
    qtree::bbox3 root_box(center, extents);
    tree.initialize(root_box, 5);

    const NodeType& root_node = tree.get_root_node();

    for ([[maybe_unused]] auto _ : state) {
        bool has_children = root_node.has_children();
        benchmark::DoNotOptimize(has_children);
    }
}

BENCHMARK(BM_QuadTreeInitialize);
BENCHMARK(BM_QuadTreeGetRootBbox);
BENCHMARK(BM_QuadTreeGetTreeDepth);
BENCHMARK(BM_QuadTreeCalculateNumberNodes);
BENCHMARK(BM_QuadTreeGetNumberNodes);
BENCHMARK(BM_QuadTreeGetRootNode);
BENCHMARK(BM_QuadTreeFindContainmentNode);
BENCHMARK(BM_NodeGetElement);
BENCHMARK(BM_NodeSetElement);
BENCHMARK(BM_NodeGetBbox);
BENCHMARK(BM_NodeGetChildAt);
BENCHMARK(BM_NodeHasChildren);

BENCHMARK_MAIN();