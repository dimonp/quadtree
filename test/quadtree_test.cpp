#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include "quadtree/quadtree.h"
#include "quadtree/quadtree_math.h"


// Test fixture for QuadTree tests
class QuadTreeTest : public ::testing::Test {
protected:
    using QuadTreeSUT = qtree::QuadTree<int, qtree::vector3, qtree::bbox3, std::vector, std::allocator>;
    using QuadTreeNodeSUT = qtree::QuadTree<int, qtree::vector3, qtree::bbox3, std::vector, std::allocator>::Node<int>;

    QuadTreeSUT sut_;
    qtree::bbox3 root_box_;

    void SetUp() override {
        qtree::vector3 min(-100.0f, -50.0f, -100.0f);
        qtree::vector3 max(100.0f, 50.0f, 100.0f);
        root_box_ = qtree::bbox3(min, max);
    }

    void TearDown() override {
    }
};

// Test initialize method with valid parameters
TEST_F(QuadTreeTest, InitializeWithValidParameters) {
    sut_.initialize(root_box_, 2);
    EXPECT_EQ(sut_.get_tree_depth(), 2);
}

// Test initialize method with invalid depth (0)
TEST_F(QuadTreeTest, InitializeWithZeroDepth) {
    ASSERT_DEATH(sut_.initialize(root_box_, 0), "");
}

// Test get_root_bbox method
TEST_F(QuadTreeTest, GetRootBbox) {
    sut_.initialize(root_box_, 2);
    const qtree::bbox3& bbox = sut_.get_root_bbox();

    EXPECT_FLOAT_EQ(bbox.get_center().x(), root_box_.get_center().x());
    EXPECT_FLOAT_EQ(bbox.get_center().y(), root_box_.get_center().y());
    EXPECT_FLOAT_EQ(bbox.get_center().z(), root_box_.get_center().z());
    EXPECT_FLOAT_EQ(bbox.get_extents().x(), root_box_.get_extents().x());
    EXPECT_FLOAT_EQ(bbox.get_extents().y(), root_box_.get_extents().y());
    EXPECT_FLOAT_EQ(bbox.get_extents().z(), root_box_.get_extents().z());
}

// Test get_tree_depth method
TEST_F(QuadTreeTest, GetTreeDepth) {
    sut_.initialize(root_box_, 4);
    EXPECT_EQ(sut_.get_tree_depth(), 4);
}

// Test calculate_number_nodes method
TEST_F(QuadTreeTest, CalculateNumberNodes) {
    sut_.initialize(root_box_, 3);

    // For level 1: (4^1 - 1) / 3 = (16 - 1) / 3 = 1
    EXPECT_EQ(sut_.calculate_number_nodes(1), 1);
    // For level 1: (4^2 - 1) / 3 = (16 - 1) / 3 = 5
    EXPECT_EQ(sut_.calculate_number_nodes(2), 5);
    // For level 2: (4^3 - 1) / 3 = (64 - 1) / 3 = 21
    EXPECT_EQ(sut_.calculate_number_nodes(3), 21);
    // For level 3: (4^4 - 1) / 3 = (256 - 1) / 3 = 85
    EXPECT_EQ(sut_.calculate_number_nodes(4), 85);
}

// Test calculate_node_index method with valid parameters
TEST_F(QuadTreeTest, CalculateNodeIndexValid) {
    sut_.initialize(root_box_, 3);

    // Root node (level 0, col 0, row 0) should be at index 0
    EXPECT_EQ(sut_.calculate_node_index(0, 0, 0), 0);
    // First level nodes
    EXPECT_EQ(sut_.calculate_node_index(1, 0, 0), 1);
    EXPECT_EQ(sut_.calculate_node_index(1, 1, 0), 2);
    EXPECT_EQ(sut_.calculate_node_index(1, 0, 1), 3);
    EXPECT_EQ(sut_.calculate_node_index(1, 1, 1), 4);
}

// Test calculate_node_index method with invalid parameters
TEST_F(QuadTreeTest, CalculateNodeIndexInvalid) {
    sut_.initialize(root_box_, 2);

    // Test with column out of bounds
    ASSERT_DEATH(sut_.calculate_node_index(1, 2, 0), "");
    // Test with row out of bounds
    ASSERT_DEATH(sut_.calculate_node_index(1, 0, 2), "");
}

// Test get_number_nodes method
TEST_F(QuadTreeTest, GetNumberNodes) {
    sut_.initialize(root_box_, 4);

    EXPECT_EQ(sut_.get_number_nodes(), 85); // Total nodes for depth 3
}

// Test get_node_by_index method with invalid indices
TEST_F(QuadTreeTest, GetNodeByIndexInvalid) {
    sut_.initialize(root_box_, 2);

    // Test with index out of bounds
    ASSERT_DEATH(sut_.get_node_by_index(100), "");
    // Test with index equal to number of nodes
    ASSERT_DEATH(sut_.get_node_by_index(sut_.get_number_nodes()), "");
}

// Test Node getters
TEST_F(QuadTreeTest, NodeGetters) {
    sut_.initialize(root_box_, 2);
    const QuadTreeNodeSUT& root_node = sut_.get_node_by_index(0);

    // Check that bbox is properly set
    const qtree::bbox3& node_bbox = root_node.get_bbox();
    EXPECT_FLOAT_EQ(node_bbox.center().x(), root_box_.center().x());
    EXPECT_FLOAT_EQ(node_bbox.center().y(), root_box_.center().y());
    EXPECT_FLOAT_EQ(node_bbox.center().z(), root_box_.center().z());
}

// Test Node::set_element and Node::get_element methods
TEST_F(QuadTreeTest, NodeElementAccess) {
    sut_.initialize(root_box_, 2);
    QuadTreeNodeSUT& root_node = sut_.get_root_node();

    // Set an element
    root_node.set_element(42);

    // Get the element back
    EXPECT_EQ(root_node.get_element(), 42);
}

// Test Node::get_child_at method
TEST_F(QuadTreeTest, NodeGetChildAt) {
    sut_.initialize(root_box_, 2);
    QuadTreeNodeSUT& root_node = sut_.get_node_by_index(0);

    // Root should have 4 children at level 1
    EXPECT_NE(root_node.get_child_at(0), nullptr);
    EXPECT_NE(root_node.get_child_at(1), nullptr);
    EXPECT_NE(root_node.get_child_at(2), nullptr);
    EXPECT_NE(root_node.get_child_at(3), nullptr);

    // Test invalid index
    ASSERT_DEATH(root_node.get_child_at(4), "");

    // Leaf nodes should have no children
    const QuadTreeNodeSUT& leaf_node = sut_.get_node_by_index(1); // First child
    EXPECT_EQ(leaf_node.get_child_at(0), nullptr);
    EXPECT_EQ(leaf_node.get_child_at(1), nullptr);
    EXPECT_EQ(leaf_node.get_child_at(2), nullptr);
    EXPECT_EQ(leaf_node.get_child_at(3), nullptr);
}

// Test find_containment_node_recursive method
TEST_F(QuadTreeTest, FindContainmentNode) {
    sut_.initialize(root_box_, 2);

    // Test with a box that fits in the root
    qtree::vector3 small_min(-10.0f, -10.0f, -10.0f);
    qtree::vector3 small_max(10.0f, 10.0f, 10.0f);
    qtree::bbox3 small_box(small_min, small_max);

    const QuadTreeNodeSUT* node = sut_.find_containment_node(small_box);
    EXPECT_NE(node, nullptr);
    EXPECT_EQ(node, &sut_.get_node_by_index(0)); // Should be root

    // Test with a box outside the root
    qtree::vector3 outside_min(190.0f, 0.0f, 0.0f);
    qtree::vector3 outside_max(210.0f, 10.0f, 10.0f);
    qtree::bbox3 outside_box(outside_min, outside_max);

    const QuadTreeNodeSUT* outside_node = sut_.find_containment_node(outside_box);
    EXPECT_EQ(outside_node, nullptr);
}

// Test Node::find_containment_node_recursive method
TEST_F(QuadTreeTest, NodeFindContainmentNode) {
    sut_.initialize(root_box_, 2);
    QuadTreeNodeSUT& root_node = sut_.get_node_by_index(0);

    // Test with a box that fits in the root
    qtree::vector3 small_min(-10.0f, -10.0f, -10.0f);
    qtree::vector3 small_max(10.0f, 10.0f, 10.0f);
    qtree::bbox3 small_box(small_min, small_max);

    const QuadTreeNodeSUT* node = root_node.find_containment_node_recursive(small_box);
    EXPECT_NE(node, nullptr);
    EXPECT_EQ(node, &sut_.get_node_by_index(0)); // Should be root
}
