#ifndef _MATH__H
#define _MATH__H

#include <Eigen/Geometry>

namespace qtree {

using line3 = Eigen::ParametrizedLine<float, 3>;
// using vector3 = Eigen::Vector3f;
using vector4 = Eigen::Vector4f;

class vector3: public Eigen::Vector3f {
public:
    vector3() = default;
    vector3(const Eigen::Vector3f& m) : Eigen::Vector3f(m) {}
    vector3(Eigen::Vector3f&& m) : Eigen::Vector3f(std::move(m)) {}
    vector3(float x, float y, float z) : Eigen::Vector3f(x, y, z) {}

    float get_x() const { return Eigen::Vector3f::x(); }
    float get_y() const { return Eigen::Vector3f::y(); }
    float get_z() const { return Eigen::Vector3f::z(); }
};

class matrix44: public Eigen::Matrix4f {
public:
    matrix44() : Eigen::Matrix4f() {}
    matrix44(const Eigen::Matrix4f& m) : Eigen::Matrix4f(m) {}

    void identity()
    {
        *this = Eigen::Matrix4f::Identity();
    }

    void inverse()
    {
        *this = Eigen::Matrix4f::inverse();
    }

    void translate(const vector3& v)
    {
        coeffRef(0, 3) += v.get_x();
        coeffRef(1, 3) += v.get_y();
        coeffRef(2, 3) += v.get_z();
    }

    void scale(const vector3& s)
    {
        for (size_t i = 0; i < 4; ++i) {
            coeffRef(0, i) *= s.get_x();
            coeffRef(1, i) *= s.get_y();
            coeffRef(2, i) *= s.get_z();
        }
    }

    // right-handed fov perspective projection matrix
    void persp_fov_rh(float fov_y, float aspect, float zn, float zf)
    {
        identity();

        float h = 1.0f / std::tanf(fov_y * 0.5f);
        float w = h / aspect;

        coeffRef(0, 0) = w;    coeffRef(1, 0) = 0.0f; coeffRef(2, 0) = 0.0f;                  coeffRef(3, 0) = 0.0f;
        coeffRef(0, 1) = 0.0f; coeffRef(1, 1) = h;    coeffRef(2, 1) = 0.0f;                  coeffRef(3, 1) = 0.0f;
        coeffRef(0, 2) = 0.0f; coeffRef(1, 2) = 0.0f; coeffRef(2, 2) = zf / (zn - zf);        coeffRef(3, 2) = -1.0f;
        coeffRef(0, 3) = 0.0f; coeffRef(1, 3) = 0.0f; coeffRef(2, 3) = zn * (zf / (zn - zf)); coeffRef(3, 3) = 0.0f;
    }

    matrix44& operator=(const Eigen::Matrix4f& other) {
        if (this != &other) {
            Base::operator=(other);
        }
        return *this;
    }
};

// class bbox3: public Eigen::AlignedBox3f {
//     enum
//     {
//         ClipLeft   = (1 << 0),
//         ClipRight  = (1 << 1),
//         ClipBottom = (1 << 2),
//         ClipTop    = (1 << 3),
//         ClipNear   = (1 << 4),
//         ClipFar    = (1 << 5),
//     };

// public:
//     enum ClipStatus
//     {
//         Outside,
//         Inside,
//         Clipped,
//     };

//     bbox3() : Eigen::AlignedBox3f() {}
//     bbox3(const Eigen::AlignedBox3f& b) : Eigen::AlignedBox3f(b) {}

//     bbox3(const vector3 &min, const vector3 &max)
//         : Eigen::AlignedBox3f(min, max) {}

//     vector3 get_min() const
//     {
//         return m_min;
//     }

//     vector3 get_max() const
//     {
//         return m_max;
//     }

//     vector3 get_center() const
//     {
//         return vector3((m_min + m_max) * 0.5f);
//     }

//     vector3 get_extents() const
//     {
//         return vector3((m_max - m_min) * 0.5f);
//     }

//     vector3 get_size() const
//     {
//         return vector3(m_max - m_min);
//     }

//     // Check for intersection with a view volume defined by a view-projection matrix.
//     inline
//     ClipStatus
//     clipstatus(const matrix44& view_projection) const
//     {
//         uint16_t and_flags = 0xffff;
//         uint16_t or_flags  = 0;
//         vector4 v0, v1;

//         for (size_t i = 0; i < 8; i++) {
//             uint8_t clip = 0;
//             v0.w() = 1.0f;

//             if (i & 1) v0.x() = m_min.x();
//             else       v0.x() = m_max.x();
//             if (i & 2) v0.y() = m_min.y();
//             else       v0.y() = m_max.y();
//             if (i & 4) v0.z() = m_min.z();
//             else       v0.z() = m_max.z();

//             v1 = view_projection * v0;

//             if (v1.x() < -v1.w())       clip |= ClipLeft;
//             else if (v1.x() > v1.w())   clip |= ClipRight;
//             if (v1.y() < -v1.w())       clip |= ClipBottom;
//             else if (v1.y() > v1.w())   clip |= ClipTop;
//             if (v1.z() < -v1.w())       clip |= ClipFar;
//             else if (v1.z() > v1.w())   clip |= ClipNear;

//             and_flags &= clip;
//             or_flags  |= clip;
//         }

//         if (0 == or_flags)       return Inside;
//         else if (0 != and_flags) return Outside;
//         else                     return Clipped;
//     }

//     inline
//     bool
//     intersect(
//         const line3& line,
//         std::vector<vector3>* isect_points = nullptr) const
//     {
//         constexpr float kRelTolerance = 1e-6f;

//         float t_near = -std::numeric_limits<float>::infinity();
//         float t_far = std::numeric_limits<float>::infinity();

//         // Iterate over all axis
//         for (size_t i = 0; i < 3; ++i) {

//             // If the line is parallel to a plane of the box
//             if (std::abs(line.direction()[i]) < kRelTolerance) {

//                 // Check if the segment origin is outside the parallel slabs
//                 if (line.origin()[i] < m_min[i] || line.origin()[i] > m_max[i]) {
//                     return false;
//                 }
//             } else {
//                 // Calculate intersection parameters for the two planes of the current axis
//                 float t1 = (m_min[i] - line.origin()[i]) / line.direction()[i];
//                 float t2 = (m_max[i] - line.origin()[i]) / line.direction()[i];

//                 // Ensure t1 is the nearer intersection and t2 is the farther
//                 if (t1 > t2) std::swap(t1, t2);

//                 // Update t_near and t_far for the intersection of all slabs
//                 t_near = std::max(t_near, t1);
//                 t_far = std::min(t_far, t2);

//                 // If tNear becomes greater than t_far, there is no intersection
//                 if (t_near > t_far) return false;
//             }
//         }

//         // Check if the intersection interval overlaps with the line segment (0 <= t <= 1)
//         if (t_near >= 0.0f && t_near <= 1.0f) {
//             if (isect_points) {
//                 auto point = line.origin() + line.direction() * t_near;
//                 isect_points->push_back(point.eval());
//             }
//         }
//         if (t_far >= 0.0f && t_far <= 1.0f) {
//             // Only add t_far if it's a distinct point and within the segment bounds
//             if (std::abs(t_far - t_near) > kRelTolerance || (t_near < 0.0f || t_near > 1.0f)) {
//                 if (isect_points) {
//                     auto point = line.origin() + line.direction() * t_far;
//                     isect_points->push_back(point.eval());
//                 }
//             }
//         }

//         // Return true if any part of the line segment [0, 1] overlaps with the box intersection [t_near, t_far]
//         return t_near <= t_far && !(t_far < 0.0f || t_near > 1.0f);
//     }

// };


class bbox3: public Eigen::AlignedBox3f {
    enum
    {
        ClipLeft   = (1 << 0),
        ClipRight  = (1 << 1),
        ClipBottom = (1 << 2),
        ClipTop    = (1 << 3),
        ClipNear   = (1 << 4),
        ClipFar    = (1 << 5),
    };

public:
    enum ClipStatus
    {
        Outside,
        Inside,
        Clipped,
    };

    bbox3() : Eigen::AlignedBox3f() {}
    bbox3(const Eigen::AlignedBox3f& b) : Eigen::AlignedBox3f(b) {}

    // bbox3(const vector3 &center, const vector3 &extents)
    //     : Eigen::AlignedBox3f(center - extents, center + extents) {}

    bbox3(const vector3 &min, const vector3 &max)
        : Eigen::AlignedBox3f(min, max) {}

    vector3 get_min() const
    {
        return Eigen::AlignedBox3f::min();
    }

    vector3 get_max() const
    {
        return Eigen::AlignedBox3f::max();
    }

    vector3 get_center() const
    {
        return vector3((m_min + m_max) * 0.5f);
    }

    vector3 get_extents() const
    {
        return vector3((m_max - m_min) * 0.5f);
    }

    vector3 get_size() const
    {
        return vector3(m_max - m_min);
    }

    // Check for intersection with a view volume defined by a view-projection matrix.
    inline
    ClipStatus
    clipstatus(const matrix44& view_projection) const
    {
        uint16_t and_flags = 0xffff;
        uint16_t or_flags  = 0;
        vector4 v0, v1;

        for (size_t i = 0; i < 8; i++) {
            uint8_t clip = 0;
            v0.w() = 1.0f;

            if (i & 1) v0.x() = m_min.x();
            else       v0.x() = m_max.x();
            if (i & 2) v0.y() = m_min.y();
            else       v0.y() = m_max.y();
            if (i & 4) v0.z() = m_min.z();
            else       v0.z() = m_max.z();

            v1 = view_projection * v0;

            if (v1.x() < -v1.w())       clip |= ClipLeft;
            else if (v1.x() > v1.w())   clip |= ClipRight;
            if (v1.y() < -v1.w())       clip |= ClipBottom;
            else if (v1.y() > v1.w())   clip |= ClipTop;
            if (v1.z() < -v1.w())       clip |= ClipFar;
            else if (v1.z() > v1.w())   clip |= ClipNear;

            and_flags &= clip;
            or_flags  |= clip;
        }

        if (0 == or_flags)       return Inside;
        else if (0 != and_flags) return Outside;
        else                     return Clipped;
    }

    inline
    bool
    test_intersection(
        const line3& line,
        std::vector<vector3>* isect_points = nullptr) const
    {
        constexpr float kRelTolerance = 1e-6f;

        float t_near = -std::numeric_limits<float>::infinity();
        float t_far = std::numeric_limits<float>::infinity();

        // Iterate over all axis
        for (size_t i = 0; i < 3; ++i) {

            // If the line is parallel to a plane of the box
            if (std::abs(line.direction()[i]) < kRelTolerance) {

                // Check if the segment origin is outside the parallel slabs
                if (line.origin()[i] < m_min[i] || line.origin()[i] > m_max[i]) {
                    return false;
                }
            } else {
                // Calculate intersection parameters for the two planes of the current axis
                float t1 = (m_min[i] - line.origin()[i]) / line.direction()[i];
                float t2 = (m_max[i] - line.origin()[i]) / line.direction()[i];

                // Ensure t1 is the nearer intersection and t2 is the farther
                if (t1 > t2) std::swap(t1, t2);

                // Update t_near and t_far for the intersection of all slabs
                t_near = std::max(t_near, t1);
                t_far = std::min(t_far, t2);

                // If tNear becomes greater than t_far, there is no intersection
                if (t_near > t_far) return false;
            }
        }

        // Check if the intersection interval overlaps with the line segment (0 <= t <= 1)
        if (t_near >= 0.0f && t_near <= 1.0f) {
            if (isect_points) {
                auto point = line.origin() + line.direction() * t_near;
                isect_points->push_back(point.eval());
            }
        }
        if (t_far >= 0.0f && t_far <= 1.0f) {
            // Only add t_far if it's a distinct point and within the segment bounds
            if (std::abs(t_far - t_near) > kRelTolerance || (t_near < 0.0f || t_near > 1.0f)) {
                if (isect_points) {
                    auto point = line.origin() + line.direction() * t_far;
                    isect_points->push_back(point.eval());
                }
            }
        }

        // Return true if any part of the line segment [0, 1] overlaps with the box intersection [t_near, t_far]
        return t_near <= t_far && !(t_far < 0.0f || t_near > 1.0f);
    }

};



}

#endif // _MATH__H