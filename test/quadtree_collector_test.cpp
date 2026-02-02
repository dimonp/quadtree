#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include "quadtree/quadtree.h"
#include "quadtree/quadtree_collector.h"
#include "quadtree/quadtree_math.h"

// Test fixture for QuadTree tests
class QuadTreeColectorTest : public ::testing::Test {
protected:
    using QuadTree = qtree::QuadTree<int, qtree::vector3, qtree::bbox3, std::vector, std::allocator>;
    using QuadTreeCollectorSUT = qtree::QuadTreeCollector<int, qtree::vector3, qtree::bbox3, std::vector, std::allocator>;

    QuadTree quad_tree_;
    qtree::bbox3 root_box_;

    void SetUp() override {
        qtree::vector3 min(-100.0f, -50.0f, -100.0f);
        qtree::vector3 max(100.0f, 50.0f, 100.0f);
        root_box_ = qtree::bbox3(min, max);
    }

    void TearDown() override {
    }
};

// Test collect_by_frustum method with a node completely outside frustum
TEST_F(QuadTreeColectorTest, CollectByFrustumOutside) {
    quad_tree_.initialize(root_box_, 2);

    auto& root_node =  quad_tree_.get_root_node();

    // Set elements in some nodes
    root_node.set_element(10);
    quad_tree_.get_node_by_index(1).set_element(20);

    // Create a projection matrix that puts all nodes outside
    qtree::matrix44 projection;
    projection.identity();
    // Move the view far away from the quadtree
    projection.translate(qtree::vector3(1000.0f, 0.0f, 0.0f));

    std::vector<int> collected;
    QuadTreeCollectorSUT::collect_by_frustum(root_node, projection, collected);

    // No elements should be collected since all nodes are outside the frustum
    EXPECT_EQ(collected.size(), 0);
}

// Test collect_by_frustum method with a node completely inside frustum
TEST_F(QuadTreeColectorTest, CollectByFrustumInside) {
    quad_tree_.initialize(root_box_, 2);

    auto& root_node =  quad_tree_.get_root_node();

    // Set elements in nodes
    root_node.set_element(10); // Root
    quad_tree_.get_node_by_index(1).set_element(20); // Child 1
    quad_tree_.get_node_by_index(2).set_element(30); // Child 2
    quad_tree_.get_node_by_index(3).set_element(40); // Child 3
    quad_tree_.get_node_by_index(4).set_element(50); // Child 4

    // Create an identity projection matrix (viewing the whole quadtree)
    qtree::matrix44 projection;
    projection.identity();

    std::vector<int> collected;
    QuadTreeCollectorSUT::collect_by_frustum(root_node, projection, collected);

    // All elements should be collected since root is completely inside
    // When root is inside, recurse_collect_nodes is called which collects all nodes
    EXPECT_EQ(collected.size(), 5); // All 5 nodes (1 root + 4 children)
    EXPECT_THAT(collected, testing::UnorderedElementsAre(10, 20, 30, 40, 50));
}

// Test collect_by_frustum method with a node partially clipped by frustum
TEST_F(QuadTreeColectorTest, CollectByFrustumClipped) {
    quad_tree_.initialize(root_box_, 2);

    auto& root_node =  quad_tree_.get_root_node();

    // Set elements in nodes
    root_node.set_element(10); // Root
    quad_tree_.get_node_by_index(1).set_element(20); // Child 1
    quad_tree_.get_node_by_index(2).set_element(30); // Child 2

    // Create a projection matrix that clips some nodes
    // Using a perspective projection that only sees part of the quadtree
    qtree::matrix44 projection;
    projection.persp_fov_rh(3.14159f/4.0f, 1.0f, 1.0f, 1000.0f); // 45-degree FOV

    // Position the camera to see only part of the quadtree
    qtree::matrix44 view;
    view.identity();
    view.translate(qtree::vector3(50.0f, 0.0f, 0.0f)); // Move camera to the side
    view.inverse();
    projection = projection * view;

    std::vector<int> collected;
    QuadTreeCollectorSUT::collect_by_frustum(root_node, projection, collected);

    // Some elements should be collected (at least the root since it's partially visible)
    // The exact number depends on the clipping, but should be >= 1
    EXPECT_EQ(collected.size(), 2);
    EXPECT_THAT(collected, testing::UnorderedElementsAre(10, 30));
}

// Test collect_by_frustum method with recursive traversal
TEST_F(QuadTreeColectorTest, CollectByFrustumRecursive) {
    quad_tree_.initialize(root_box_, 2);

    auto& root_node =  quad_tree_.get_root_node();

    // Set elements in all nodes
    root_node.set_element(10); // Root
    quad_tree_.get_node_by_index(1).set_element(20); // Child 1
    quad_tree_.get_node_by_index(2).set_element(30); // Child 2
    quad_tree_.get_node_by_index(3).set_element(40); // Child 3
    quad_tree_.get_node_by_index(4).set_element(50); // Child 4

    // Create a projection matrix that clips the root but includes children
    // This is a bit tricky to set up, so we'll use a simple case where
    // the root is clipped but some children might be included
    qtree::matrix44 projection;
    projection.identity();
    // Apply a small scale to make the frustum smaller than the root
    projection.scale(qtree::vector3(0.5f, 1.0f, 0.5f));

    std::vector<int> collected;
    QuadTreeCollectorSUT::collect_by_frustum(root_node, projection, collected);

    // At least the root should be collected since it's the entry point
    EXPECT_GE(collected.size(), 1);
    EXPECT_THAT(collected, testing::Contains(10));
}

// Test collect_by_line_intersect method with a line that intersects root only
TEST_F(QuadTreeColectorTest, CollectByLineIntersectRootOnly) {
    quad_tree_.initialize(root_box_, 2);

    auto& root_node = quad_tree_.get_root_node();

    // Set elements in nodes
    root_node.set_element(10); // Root
    quad_tree_.get_node_by_index(1).set_element(20); // Child 1

    // Create a line that goes through the root and children without elements
    qtree::vector3 start(-5.0f, 0.0f, 10.0f);
    qtree::vector3 end(10.0f, 0.0f, -5.0f);
    qtree::line3 line(start, end);

    std::vector<int> collected;
    QuadTreeCollectorSUT::collect_by_line_intersect(root_node, line, collected);

    // Only the root element should be collected
    EXPECT_EQ(collected.size(), 1);
    EXPECT_THAT(collected, testing::ElementsAre(10));
}

// Test collect_by_line_intersect method with a line that intersects multiple nodes
TEST_F(QuadTreeColectorTest, CollectByLineIntersectMultipleNodes) {
    quad_tree_.initialize(root_box_, 2);

    auto& root_node = quad_tree_.get_root_node();

    // Set elements in nodes
    root_node.set_element(10); // Root
    quad_tree_.get_node_by_index(1).set_element(20); // Child 1
    quad_tree_.get_node_by_index(2).set_element(30); // Child 2
    quad_tree_.get_node_by_index(3).set_element(40); // Child 3
    quad_tree_.get_node_by_index(4).set_element(50); // Child 4

    // Create a line that goes diagonally through the quadtree, intersecting multiple nodes
    qtree::vector3 start(-150.0f, 0.0f, -150.0f);
    qtree::vector3 end(150.0f, 0.0f, 150.0f);
    qtree::line3 line(start, end);

    std::vector<int> collected;
    QuadTreeCollectorSUT::collect_by_line_intersect(root_node, line, collected);

    // All elements should be collected since the line intersects the root and goes through all children
    EXPECT_EQ(collected.size(), 5);
    EXPECT_THAT(collected, testing::UnorderedElementsAre(10, 20, 30, 40, 50));
}

// Test collect_by_line_intersect method with a line that doesn't intersect any nodes
TEST_F(QuadTreeColectorTest, CollectByLineIntersectNoIntersection) {
    quad_tree_.initialize(root_box_, 2);

    auto& root_node = quad_tree_.get_root_node();

    // Set elements in nodes
    root_node.set_element(10); // Root
    quad_tree_.get_node_by_index(1).set_element(20); // Child 1

    // Create a line that is completely outside the quadtree
    qtree::vector3 start(-200.0f, 0.0f, -200.0f);
    qtree::vector3 end(-150.0f, 0.0f, -150.0f);
    qtree::line3 line(start, end);

    std::vector<int> collected;
    QuadTreeCollectorSUT::collect_by_line_intersect(root_node, line, collected);

    // No elements should be collected since the line doesn't intersect any nodes
    EXPECT_EQ(collected.size(), 0);
}

// Test collect_by_line_intersect method with a line that intersects leaf nodes
TEST_F(QuadTreeColectorTest, CollectByLineIntersectLeafNodes) {
    quad_tree_.initialize(root_box_, 2);

    auto& root_node = quad_tree_.get_root_node();

    // Set elements in leaf nodes only (children of the root)
    quad_tree_.get_node_by_index(1).set_element(20); // Child 1
    quad_tree_.get_node_by_index(3).set_element(40); // Child 3

    // Create a line that intersects specific child nodes
    // Based on the quadtree structure, child 1 is at (0,0) to (100,100) and child 3 is at (-100,-100) to (0,0)
    // So a line from (-50,-50) to (50,50) should intersect child 1 and child 3
    qtree::vector3 start(-50.0f, 0.0f, -50.0f);
    qtree::vector3 end(50.0f, 0.0f, 50.0f);
    qtree::line3 line(start, end);

    std::vector<int> collected;
    QuadTreeCollectorSUT::collect_by_line_intersect(root_node, line, collected);

    // Only elements from child 1 and child 3 should be collected
    EXPECT_EQ(collected.size(), 2);
    EXPECT_THAT(collected, testing::UnorderedElementsAre(20, 40));
}

// Test collect_by_line_intersect method with a line that starts inside a node
TEST_F(QuadTreeColectorTest, CollectByLineIntersectStartInside) {
    quad_tree_.initialize(root_box_, 2);

    auto& root_node = quad_tree_.get_root_node();

    // Set elements in nodes
    root_node.set_element(10); // Root
    quad_tree_.get_node_by_index(1).set_element(20); // Child 1

    // Create a line that starts inside the root node
    qtree::vector3 start(0.0f, 0.0f, 0.0f); // Center of root
    qtree::vector3 end(50.0f, 0.0f, 50.0f);
    qtree::line3 line(start, end);

    std::vector<int> collected;
    QuadTreeCollectorSUT::collect_by_line_intersect(root_node, line, collected);

    // The root element should be collected since the line starts inside it
    EXPECT_GE(collected.size(), 1);
    EXPECT_THAT(collected, testing::Contains(10));
}
