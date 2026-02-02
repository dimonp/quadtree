#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "quadtree/quadtree.h"
#include "quadtree/quadtree_math.h"


// Test fixture for QuadTree tests
class QuadTreeAdvTest : public ::testing::Test {
protected:

    struct TestPayload {
        int value_one;
        double value_two;

        operator bool() const
        {
            return false;
        }
    };

    using QuadTreeElement = TestPayload;
    using QuadTreeSUT = qtree::QuadTree<QuadTreeElement, qtree::vector3, qtree::bbox3, std::vector, std::allocator>;
    using QuadTreeNodeSUT = qtree::QuadTree<QuadTreeElement, qtree::vector3, qtree::bbox3, std::vector, std::allocator>::Node<QuadTreeElement>;

    QuadTreeSUT sut_;
    qtree::bbox3 root_box_;

    void SetUp() override {
        qtree::vector3 center(0.0f, 0.0f, 0.0f);
        qtree::vector3 extents(100.0f, 50.0f, 100.0f);
        root_box_ = qtree::bbox3(center, extents);
    }

    void TearDown() override {
    }
};


// Test Node::set_element and Node::get_element methods
TEST_F(QuadTreeAdvTest, NodeElementAccess) {
    sut_.initialize(root_box_, 2);
    QuadTreeNodeSUT& root_node = sut_.get_root_node();

    // Set an element
    TestPayload payload {1, 2.0};
    root_node.set_element(payload);

    // Get the element back
    EXPECT_EQ(root_node.get_element(), payload);
}

