#ifndef _QUADTREE_COLLECTOR__H
#define _QUADTREE_COLLECTOR__H

#include "quadtree.h"

namespace qtree {

/**
 * @file quadtree_collector.h
 * @brief A collector for quadtree nodes based on frustum culling.
 *
 * This class provides functionality to collect elements from a quadtree
 * that are within a given view frustum, using efficient culling techniques.
 */

template<
    typename T,
    typename VEC3,
    typename BBOX3,
    template<typename, typename> typename ContainerT,
    template<typename> class AllocatorT>
class QuadTreeCollector {
public:
    /**
     * @brief Collect quadtree nodes that are within the view frustum.
     *
     * @param node The root node to start collection from
     * @param projection The view-projection matrix for frustum culling
     * @param collected Vector to store collected elements
     */
    template<typename MAT44>
    static void collect_by_frustum( const typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>& node,
                                    const MAT44& projection,
                                    ContainerT<T, AllocatorT<T>>& collected);

    /**
     * @brief Collect quadtree nodes that intersect with a line.
     *
     * @param node The root node to start collection from
     * @param line The line to test for intersection
     * @param collected Vector to store collected elements
     */
    template<typename LINE3>
    static void collect_by_line_intersect(  const typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>& node,
                                            const LINE3& line,
                                            ContainerT<T, AllocatorT<T>>& collected);

private:

    template<typename MAT44>
    static void recurse_collect_by_frustum( const typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>& node,
                                            const MAT44& projection,
                                            ContainerT<T, AllocatorT<T>>& collected);

    static void recurse_collect_all_nodes(  const typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>& node,
                                            ContainerT<T, AllocatorT<T>>& collected);


    template<typename LINE3>
    static void recurse_line_intersect( const typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>& node,
                                        const LINE3& line,
                                        ContainerT<T, AllocatorT<T>>& collected);

};

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
template<typename MAT44>
inline void
QuadTreeCollector<T, VEC3, BBOX3, ContainerT, AllocatorT>::collect_by_frustum(
    const typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>& node,
    const MAT44& projection,
    ContainerT<T, AllocatorT<T>>& collected)
{
    collected.clear();
    recurse_collect_by_frustum(node, projection, collected);
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
template<typename LINE3>
inline void
QuadTreeCollector<T, VEC3, BBOX3, ContainerT, AllocatorT>::collect_by_line_intersect(const typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>& node,
                                                const LINE3& line,
                                                ContainerT<T, AllocatorT<T>>& collected)
{
    collected.clear();
    recurse_line_intersect(node, line, collected);
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
template<typename MAT44>
inline void
QuadTreeCollector<T, VEC3, BBOX3, ContainerT, AllocatorT>::recurse_collect_by_frustum( const typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>& node,
                                                  const MAT44& projection,
                                                  ContainerT<T, AllocatorT<T>>& collected)
{
    // Determine the clipping status of the node's bounding box against the frustum
    const typename BBOX3::ClipStatus clip_status = node.get_bbox().clipstatus(projection);

    // If the node is completely outside the frustum, skip it and its children
    if (clip_status == BBOX3::Outside) {
        return;
    }

    // If the node is completely inside the frustum, collect all its descendants
    if (clip_status == BBOX3::Inside) {
        recurse_collect_all_nodes(node, collected);
    } else {
        // Add the element if it exists
        auto &element = node.get_element();
        if (element) {
            collected.push_back(element);
        }

        // For partially clipped nodes, check the node itself and then its children
        // Add the element if it exists
        // Note: This assumes T can be evaluated in a boolean context or get_element()
        // returns a meaningful value even when "empty"

        // Recursively process children if they exist
        if (node.has_children()) {
            for(size_t i = 0; i < 4; ++i) {
                recurse_collect_by_frustum(*node.get_child_at(i), projection, collected);
            }
        }
    }
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
inline void
QuadTreeCollector<T, VEC3, BBOX3, ContainerT, AllocatorT>::recurse_collect_all_nodes( const typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>& node,
                                                 ContainerT<T, AllocatorT<T>>& collected)
{
    // Add the element if it exists
    auto &element = node.get_element();
    if (element) {
        collected.push_back(element);
    }

    // Recursively process children if they exist
    if (node.has_children()) {
        for(size_t i = 0; i < 4; ++i) {
            recurse_collect_all_nodes(*node.get_child_at(i), collected);
        }
    }
}

template<typename T, typename VEC3, typename BBOX3, template<typename, typename> typename ContainerT, template<typename> class AllocatorT>
template<typename LINE3>
inline void
QuadTreeCollector<T, VEC3, BBOX3, ContainerT, AllocatorT>::recurse_line_intersect(const typename QuadTree<T, VEC3, BBOX3, ContainerT, AllocatorT>::template Node<T>& node,
                                             const LINE3& line,
                                             ContainerT<T, AllocatorT<T>>& collected)
{
    if (node.get_bbox().test_intersection(line)) {

        auto &element = node.get_element();
        if (element) {
            collected.push_back(element);
        }

        if (node.has_children()) {
            for (size_t i = 0; i < 4; ++i) {
                recurse_line_intersect(*node.get_child_at(i), line, collected);
            }
        }
    }
}

}

#endif // _QUADTREE_COLLECTOR__H
