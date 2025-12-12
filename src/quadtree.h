#ifndef _QUADTREE__H
#define _QUADTREE__H

#include <cstdint>
#include <cstdlib>
#include <type_traits>
#include <gsl/gsl>

using std::uint8_t;
using std::uint16_t;
using std::uint32_t;

#ifndef LOG_ERROR
#define LOG_ERROR(...) std::fprintf(stderr, __VA_ARGS__)
#endif

#ifndef LOG_INFO
#define LOG_INFO(...) std::fprintf(stdout, __VA_ARGS__)
#endif

namespace qtree {

/**
 * @brief A QuadTree implementation for spatial partitioning.
 *
 * This class implements a quadtree data structure for efficient spatial queries
 * in 3D space. The tree is composed of nodes that recursively subdivide a
 * bounding volume into four quadrants at each level.
 *
 * @tparam T The type of data stored in each node
 * @tparam VEC3 3D vector type used for positions
 * @tparam BBOX3 3D bounding box type
 * @tparam ContainerT Container type for storing nodes
 * @tparam AllocatorT Allocator type for nodes
 */

template<
    typename T,
    typename VEC3,
    typename BBOX3,
    template<typename, typename> typename ContainerT,
    template<typename> class AllocatorT>
class QuadTree {
public:
    static_assert(std::is_convertible_v<T, bool>);

    template <typename U> class Node;

    QuadTree() = default;

    QuadTree(const QuadTree&) = delete;
    void operator = (const QuadTree&) = delete;

    /**
     * @brief Initialize the quadtree with a bounding box and depth.
     *
     * @param box The bounding box for the root node
     * @param depth The maximum depth of the tree (must be > 0)
     */
    void initialize(const BBOX3& box, uint8_t depth);

    /**
     * @brief Reset the quadtree to its initial state.
     *
     * Clears all nodes and resets the tree.
     */
    void reset();

    /**
     * @brief Get the root bounding box.
     * @return const reference to the root bounding box
     */
    const BBOX3& get_root_bbox() const;

    /**
     * @brief Get the tree depth.
     * @return The maximum depth of the tree
     */
    uint8_t get_tree_depth() const;

    /**
     * @brief Calculate the total number of nodes up to a given level.
     *
     * @param level The level to calculate nodes for
     * @return The number of nodes up to the given level
     */
    size_t calculate_number_nodes(uint8_t level) const;

    /**
     * @brief Calculate the linear index for a node at a specific level, column, and row.
     *
     * @param level The level of the node
     * @param col The column of the node
     * @param row The row of the node
     * @return The linear index of the node
     */
    size_t calculate_node_index(uint8_t level, uint16_t col, uint16_t row) const;

    /**
     * @brief Get the total number of nodes in the tree.
     * @return The number of nodes in the tree
     */
    size_t get_number_nodes() const;

    /**
     * @brief Get the root node of the quadtree.
     * @return const reference to the root node
     */
    const Node<T>& get_root_node() const;
    Node<T>& get_root_node();

    /**
     * @brief Get a node by its index
     *
     * @param index The index of the node
     * @return const reference to the node
     */
    const Node<T>& get_node_by_index(size_t index) const;
    Node<T>& get_node_by_index(size_t index);

    /**
     * @brief Find the smallest node that completely contains the given bounding box.
     *
     * @param box The bounding box to find containment for
     * @return Pointer to the containing node, or nullptr if not found
     */
    Node<T>* find_containment_node(const BBOX3& box);

    /**
     * @brief A node in the quadtree.
     */
    /**
     * @brief A node in the quadtree that represents a spatial region.
     *
     * Each node represents a rectangular region in 3D space and can store data
     * of type T. Nodes can have up to four children that subdivide the parent's
     * region into quadrants.
     *
     * @tparam U The type of data stored in this node (same as T from the parent QuadTree)
     */
    template <typename U>
    class Node {
    public:

        /**
         * @brief Initialize the node with tree parameters.
         *
         * @param tree Pointer to the parent tree
         * @param level The level of this node
         * @param col The column of this node
         * @param row The row of this node
         */
        void initialize(QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>* tree, uint8_t level, uint16_t col, uint16_t row);

        /**
         * @brief Get the level of this node.
         * @return The level
         */
        // uint8_t get_level() const;

        /**
         * @brief Get the column of this node.
         * @return The column
         */
        // uint16_t get_column() const;

        /**
         * @brief Get the row of this node.
         * @return The row
         */
        // uint16_t get_row() const;

        /**
         * @brief Get the bounding box of this node.
         * @return const reference to the bounding box
         */
        const BBOX3& get_bbox() const;

        /**
         * @brief Find the smallest child node that contains the given box.
         *
         * @param box The box to find containment for
         * @return Pointer to the containing node, or nullptr if not found
         */
        Node* find_containment_node_recursive(const BBOX3& box);

        /**
         * @brief Set the element stored in this node.
         *
         * @param element The element to store
         */
        void set_element(const T& element);

        /**
         * @brief Get the element stored in this node.
         *
         * @return const reference to the element
         */
        const T& get_element() const;

        /**
         * @brief Get a child node by index.
         *
         * @param index The index of the child (0-3)
         * @return Pointer to the child node, or nullptr if index is invalid or child doesn't exist
         */
        Node* get_child_at(size_t index);
        const Node* get_child_at(size_t index) const;

        /**
        * @brief Check if a node has any children (is not a leaf).
        *
        * @return true if node has children, false otherwise
        */
        bool has_children() const;

    private:
        U element_ = {};

        Node* children_[4] = { nullptr, nullptr, nullptr, nullptr };
        BBOX3 bbox_;

        friend class QuadTree;
    };

private:

    ContainerT<Node<T>, AllocatorT<Node<T>>> node_array_;
    BBOX3 root_bbox_;
    uint8_t tree_depth_ = 0;
    VEC3 base_node_size_;

    friend class Node<T>;
};

// QuadTree implementation

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
inline void
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::initialize(const BBOX3& box, uint8_t depth) {
    Expects(depth > 0 && "Tree depth must be greater than 0");

    tree_depth_ = depth;
    root_bbox_ = box;

    float base_dimension = static_cast<float>(1 << (tree_depth_ - 1));
    base_node_size_ = VEC3( root_bbox_.get_size().get_x() / base_dimension,
                            root_bbox_.get_size().get_y(),
                            root_bbox_.get_size().get_z() / base_dimension);

    size_t num_nodes = calculate_number_nodes(tree_depth_);
    node_array_.resize(num_nodes);
    node_array_[0].initialize(this, 0, 0, 0);
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
inline void
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::reset()
{
    node_array_.clear();
    // base_node_size_ = VEC3();
    root_bbox_ = BBOX3();
    tree_depth_ = 0;
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
inline uint8_t
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::get_tree_depth() const
{
    return tree_depth_;
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
inline const BBOX3&
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::get_root_bbox() const
{
    return root_bbox_;
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
inline size_t
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::calculate_number_nodes(uint8_t level) const
{
    // Formula: (4^(level+1) - 1) / 3
    // Using bit shifting for efficiency: 4^n = 2^(2n) = 1 << (2n)
    return ((1 << (2 * level)) - 1) / 3;
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>

inline size_t
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::get_number_nodes() const
{
    return node_array_.size();
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
inline size_t
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::calculate_node_index(uint8_t level, uint16_t col, uint16_t row) const
{
    Expects((col >= 0) && (col < (1 << level)));
    Expects((row >= 0) && (row < (1 << level)));

    size_t parent_nodes = calculate_number_nodes(level);
    return parent_nodes + (static_cast<size_t>(row) << level) + col;
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
inline typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>*
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::find_containment_node(const BBOX3& box)
{
    Expects(!node_array_.empty());
    return get_root_node().find_containment_node_recursive(box);
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
inline const typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>&
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::get_root_node() const
{
    Expects(node_array_.size() > 0);
    return node_array_[0];
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
inline typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>&
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::get_root_node()
{
    Expects(node_array_.size() > 0);
    return node_array_[0];
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
inline const typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>&
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::get_node_by_index(size_t index) const
{
    Expects(index < node_array_.size() && "Node index out of bounds");
    return node_array_[index];
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
inline typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>&
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::get_node_by_index(size_t index)
{
    Expects(index < node_array_.size() && "Node index out of bounds");
    return node_array_[index];
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
template <typename U>
void
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::Node<U>::initialize(QuadTree* tree, uint8_t level, uint16_t col, uint16_t row)
{
    Expects(tree != nullptr && "Tree pointer cannot be null");
    Expects(col < (1 << level) && row < (1 << level) && "Column or row index out of bounds for the specified level");

    // Update bounding box
    float level_factor = static_cast<float>(1 << (tree->tree_depth_ - 1 - level));

    const VEC3& base_size = tree->base_node_size_;
    const BBOX3& tree_bbox = tree->root_bbox_;
    VEC3 tree_min = tree_bbox.get_min();
    VEC3 tree_max = tree_bbox.get_max();

    VEC3 min = VEC3( tree_min.get_x() + (col * level_factor * base_size.get_x()),
                     tree_min.get_y(),
                     tree_min.get_z() + (row * level_factor * base_size.get_z()));

    VEC3 max = VEC3( tree_min.get_x() + ((col + 1) * level_factor * base_size.get_x()),
                     tree_max.get_y(),
                     tree_min.get_z() + ((row + 1) * level_factor * base_size.get_z()));

    bbox_ = BBOX3(min, max);

    // Recursively initialize children
    uint8_t child_level = level + 1;
    if (child_level < tree->tree_depth_) {
        for (size_t i = 0; i < 4; ++i) {
            uint16_t child_col = static_cast<uint16_t>(2 * col + (i & 1));
            uint16_t child_row = static_cast<uint16_t>(2 * row + ((i & 2) >> 1));
            size_t child_index = tree->calculate_node_index(child_level, child_col, child_row);
            children_[i] = &(tree->node_array_[child_index]);
            children_[i]->initialize(tree, child_level, child_col, child_row);
        }
    }
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
template <typename U>
typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<U>*
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::Node<U>::find_containment_node_recursive(const BBOX3& check_box)
{
    const VEC3& v_min_cb = check_box.get_min();
    const VEC3& v_max_cb = check_box.get_max();
    const VEC3& v_min = bbox_.get_min();
    const VEC3& v_max = bbox_.get_max();

    if (v_min_cb.get_x() < v_min.get_x() || v_max_cb.get_x() > v_max.get_x() ||
        v_min_cb.get_y() < v_min.get_y() || v_max_cb.get_y() > v_max.get_y() ||
        v_min_cb.get_z() < v_min.get_z() || v_max_cb.get_z() > v_max.get_z()) {

        return nullptr;
    }

    // Check children first
    if (has_children()) {
        for (Node* node_ptr : children_) {
            Node* contain_node = node_ptr->find_containment_node_recursive(check_box);
            if (contain_node != nullptr) {
                return contain_node;
            }
        }
    }

    // Not contained in children, but still contained in this node
    return this;
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
template <typename U>
inline const BBOX3&
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::Node<U>::get_bbox() const
{
    return bbox_;
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
template <typename U>
inline void
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::Node<U>::set_element(const T& element)
{
    element_ = element;
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
template <typename U>
inline const T&
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::Node<U>::get_element() const
{
    return element_;
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
template <typename U>
inline typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<U>*
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::Node<U>::get_child_at(size_t index)
{
    Expects(index < 4);
    return children_[index];
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
template <typename U>
const inline typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<U>*
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::Node<U>::get_child_at(size_t index) const
{
    Expects(index < 4);
    return children_[index];
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
template <typename U>
inline bool
QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::Node<U>::has_children() const
{
    // A node has children if its first child is not null
    // This assumes that children are either all allocated or all nullptr
    return get_child_at(0) != nullptr;
}

}

#endif // _QUADTREE__H

