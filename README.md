# C++ Quad tree Implementation

This project provides a generic, efficient implementation of a QuadTree data structure for spatial partitioning in 3D space. The implementation is provides features for frustum culling and line intersection queries.

## Features

- Generic QuadTree implementation that can store any data type
- Efficient spatial partitioning in 3D space
- Frustum culling for view-dependent queries
- Line intersection queries for ray casting
- Integration with Eigen for mathematical operations
- Comprehensive benchmark suite
- Full test coverage with Google Test

## Project Structure

```
├── src/                 # Core QuadTree implementation
├── math/                # Mathematical types and operations using Eigen
├── test/                # Unit tests using Google Test
├── benchmark/            # Performance benchmarks using Google Benchmark
├── CMakeLists.txt       # Main CMake configuration
└── README.md            # This file
```

## Dependencies

- C++17 compatible compiler
- Eigen 3.4+
- Google Test (for tests)
- Google Benchmark (for benchmarks)

## Building

This project uses CMake for building. To build the project:

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

To build without tests and benchmarks:

```bash
mkdir build
cd build
cmake -DBUILD_TESTING=OFF ..
cmake --build .
```

## Usage

### Basic QuadTree Usage

```cpp
#include "quadtree.h"
#include "quadtree_math.h"

// Define types
using QuadTreeType = qtree::QuadTree<int, qtree::vector3, qtree::bbox3, std::vector, std::allocator>;


// Create and initialize a quadtree
QuadTreeType tree;
qtree::vector3 min(-100.0f, -50.0f, -100.0f);
qtree::vector3 max(100.0f, 50.0f, 100.0f);
qtree::bbox3 root_box(min, max);
tree.initialize(root_box, 5); // 5 levels deep

// Find a node that contains a specific bounding box
qtree::vector3 box_min(5.0f, -5.0f, 5.0f);
qtree::vector3 box_max(15.0f, 5.0f, 15.0f);
qtree::bbox3 search_box(box_min, box_max);
auto* node = tree.find_containment_node(search_box);

if (node) {
    node->set_element(42); // Store data in the node
    int value = node->get_element(); // Retrieve data from the node
}
```

### Frustum Culling

```cpp
#include "quadtree_collector.h"

// Define types
using QuadTreeCollectorType = qtree::QuadTreeCollector<int, qtree::vector3, qtree::bbox3, std::vector, std::allocator>;

// Create a projection matrix for frustum culling
qtree::matrix44 projection;
projection.persp_fov_rh(45.0f, 1.0f, 0.1f, 1000.0f);
projection.translate(qtree::vector3(0.0f, 0.0f, -50.0f));

// Collect elements that are within the view frustum
std::vector<int> collected;
QuadTreeCollectorType::collect_by_frustum(tree.get_root_node(), projection, collected);
```

### Line Intersection Queries

```cpp
#include "quadtree_collector.h"

// Create a line for intersection testing
qtree::vector3 origin(-50.0f, 0.0f, -50.0f);
qtree::vector3 direction(100.0f, 0.0f, 100.0f);
qtree::line3 line(origin, direction);

// Collect elements that intersect with the line
std::vector<int> collected;
QuadTreeCollectorType::collect_by_line_intersect(tree.get_root_node(), line, collected);
```

## API Documentation

### QuadTree Class

The main QuadTree class is a template class with the following parameters:
- `T`: The type of data stored in each node
- `VEC3`: 3D vector type (default: qtree::vector3)
- `BBOX3`: 3D bounding box type (default: qtree::bbox3)
- `ContainerT`: Container type for storing nodes (default: std::vector)
- `AllocatorT`: Allocator type for nodes (default: std::allocator)

#### Key Methods

- `void initialize(const BBOX3& box, uint8_t depth)`: Initialize the quadtree with a bounding box and depth
- `const BBOX3& get_root_bbox() const`: Get the root bounding box
- `uint8_t get_tree_depth() const`: Get the tree depth
- `Node<T>* find_containment_node(const BBOX3& box)`: Find the smallest node that contains the given box
- `const Node<T>& get_root_node() const`: Get the root node
- `const Node<T>& get_node_by_index(size_t index) const`: Get a node by its index

### Node Class

Each node in the quadtree represents a spatial region and can store data of type T.

#### Key Methods

- `const BBOX3& get_bbox() const`: Get the bounding box of this node
- `void set_element(const T& element)`: Set the element stored in this node
- `const T& get_element() const`: Get the element stored in this node
- `Node* get_child_at(size_t index)`: Get a child node by index (0-3)
- `bool has_children() const`: Check if the node has children

### QuadTreeCollector Class

Provides functionality for collecting elements from a quadtree based on various criteria.

#### Key Methods

- `static void collect_by_frustum(const Node<T>& node, const MAT44& projection, ContainerT<T, AllocatorT<T>>& collected)`: Collect nodes within a view frustum
- `static void collect_by_line_intersect(const Node<T>& node, const LINE3& line, ContainerT<T, AllocatorT<T>>& collected)`: Collect nodes that intersect with a line

## Testing

The project includes comprehensive unit tests using Google Test. To run the tests:

```bash
cd build
ctest .
```

Or to run the tests directly:

```bash
cd build
./test/quadtree_tests
```

## Benchmarking

The project includes performance benchmarks using Google Benchmark. To run the benchmarks:

```bash
cd build
./benchmark/quadtree_benchmark
./benchmark/quadtree_collector_benchmark
```

### Benchmark Results

The benchmarks measure performance of various operations:

- QuadTree initialization
- Node access operations
- Frustum culling collection
- Line intersection collection

## License

This project is licensed under the MIT License - see the LICENSE file for details.