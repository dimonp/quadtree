#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "quadtree.h"
#include "quadtree_math.h"

// Test fixture for comprehensive QuadTree tests
class QuadTreeComprehensiveTest : public ::testing::Test {
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

// Test reset method
TEST_F(QuadTreeComprehensiveTest, Reset) {
    // Initialize the tree
    sut_.initialize(root_box_, 3);
    EXPECT_EQ(sut_.get_tree_depth(), 3);
    EXPECT_EQ(sut_.get_number_nodes(), 21); // For depth 3: (4^3 - 1) / 3 = 21

    // Reset the tree
    sut_.reset();

    // Check that the tree is properly reset
    EXPECT_EQ(sut_.get_tree_depth(), 0);
    EXPECT_EQ(sut_.get_number_nodes(), 0);
}

// Test Node::has_children method for internal nodes
TEST_F(QuadTreeComprehensiveTest, NodeHasChildrenInternal) {
    sut_.initialize(root_box_, 3);
    QuadTreeNodeSUT& root_node = sut_.get_root_node();

    // Root node should have children
    EXPECT_TRUE(root_node.has_children());

    // First level nodes should also have children
    QuadTreeNodeSUT* child_node = root_node.get_child_at(0);
    ASSERT_NE(child_node, nullptr);
    EXPECT_TRUE(child_node->has_children());

    // Second level nodes should not have children (they are leaves)
    QuadTreeNodeSUT* grandchild_node = child_node->get_child_at(0);
    ASSERT_NE(grandchild_node, nullptr);
    EXPECT_FALSE(grandchild_node->has_children());
}

// Test Node::has_children method for leaf nodes
TEST_F(QuadTreeComprehensiveTest, NodeHasChildrenLeaf) {
    sut_.initialize(root_box_, 2);

    // Get a leaf node (first child of root)
    QuadTreeNodeSUT& leaf_node = sut_.get_node_by_index(1);

    // Leaf node should not have children
    EXPECT_FALSE(leaf_node.has_children());
}

// Test Node::optimize_recursive method with empty nodes
TEST_F(QuadTreeComprehensiveTest, OptimizeRecursiveEmptyNodes) {
    sut_.initialize(root_box_, 3);

    // Initially all nodes should be empty (default int value is 0, which is falsy)
    QuadTreeNodeSUT& root_node = sut_.get_root_node();

    // Optimize should return false since no nodes have elements
    EXPECT_FALSE(root_node.optimize_recursive());

    // Children should be empty
    EXPECT_FALSE(root_node.has_children());
    for (size_t i = 0; i < 4; ++i) {
        QuadTreeNodeSUT* child = root_node.get_child_at(i);
        ASSERT_EQ(child, nullptr);
    }
}

// Test Node::optimize_recursive method with populated nodes
TEST_F(QuadTreeComprehensiveTest, OptimizeRecursiveWithElements) {
    sut_.initialize(root_box_, 3);

    // Set an element in a leaf node
    QuadTreeNodeSUT& leaf_node = sut_.get_node_by_index(5); // A leaf node at level 2
    leaf_node.set_element(42);

    // Optimize should return true since a node has an element
    QuadTreeNodeSUT& root_node = sut_.get_root_node();
    EXPECT_TRUE(root_node.optimize_recursive());

    // Children should still exist since a descendant has an element
    EXPECT_TRUE(root_node.has_children());
}

// Test Node::optimize_recursive method cleans up empty branches
TEST_F(QuadTreeComprehensiveTest, OptimizeRecursiveCleanup) {
    sut_.initialize(root_box_, 3);

    // Set an element in a leaf node
    QuadTreeNodeSUT& leaf_node = sut_.get_node_by_index(5); // A leaf node at level 2
    leaf_node.set_element(42);

    // Optimize the tree
    QuadTreeNodeSUT& root_node = sut_.get_root_node();
    EXPECT_TRUE(root_node.optimize_recursive());

    // Check that nodes without elements have had their children cleared
    // Get a sibling of the node with the element (should have no children after optimization)
    QuadTreeNodeSUT& sibling_node = sut_.get_node_by_index(6);
    EXPECT_FALSE(sibling_node.has_children());
}

// Test calculate_number_nodes with edge cases
TEST_F(QuadTreeComprehensiveTest, CalculateNumberNodesEdgeCases) {
    sut_.initialize(root_box_, 1);

    // For level 0: (4^1 - 1) / 3 = 1
    EXPECT_EQ(sut_.calculate_number_nodes(1), 1);

    // For level 1: (4^2 - 1) / 3 = 5
    EXPECT_EQ(sut_.calculate_number_nodes(2), 5);
}

// Test calculate_node_index with edge cases
TEST_F(QuadTreeComprehensiveTest, CalculateNodeIndexEdgeCases) {
    sut_.initialize(root_box_, 4);

    // Test with maximum valid indices for level 3
    uint8_t level = 3;
    uint16_t max_col = (1 << level) - 1;  // 7 for level 3
    uint16_t max_row = (1 << level) - 1;  // 7 for level 3

    // This should not cause an assertion failure
    size_t index = sut_.calculate_node_index(level, max_col, max_row);
    EXPECT_GT(index, 0);
}

// Test get_node_by_index with valid indices
TEST_F(QuadTreeComprehensiveTest, GetNodeByIndexValid) {
    sut_.initialize(root_box_, 3);

    // Test getting root node
    const QuadTreeNodeSUT& root_node = sut_.get_node_by_index(0);
    EXPECT_EQ(&root_node, &sut_.get_root_node());

    // Test getting a child node
    const QuadTreeNodeSUT& child_node = sut_.get_node_by_index(1);
    EXPECT_NE(&child_node, &sut_.get_root_node());
}