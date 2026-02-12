/**
 * projection.cpp
 */

#include <vector>
#include <cmath>
#include <limits>
#include <variant>
#include <optional>
#include <CGAL/Aff_transformation_3.h>
#include "common.h"

using namespace Ink3d;

// Helper: Construct a LookAt matrix (World -> Camera)
// Eye at (1,1,1), looking at (0,0,0), Up is roughly Z
CGAL::Aff_transformation_3<K> look_at_transform(const Point_3& eye, const Point_3& center, const Vector_3& up) 
{
    Vector_3 f = (center - eye); // Forward
    f = f / std::sqrt(f.squared_length());
    
    Vector_3 s = CGAL::cross_product(f, up); // Side
    s = s / std::sqrt(s.squared_length());
    
    Vector_3 u = CGAL::cross_product(s, f); // Recomputed Up
    u = u / std::sqrt(u.squared_length());

    // Construct rotation matrix rows
    return CGAL::Aff_transformation_3<K>(
        s.x(), s.y(), s.z(), -s * (eye - CGAL::ORIGIN),
        u.x(), u.y(), u.z(), -u * (eye - CGAL::ORIGIN),
        -f.x(), -f.y(), -f.z(), f * (eye - CGAL::ORIGIN)
    );
}

// Helper: Project 3D Camera-space point to 2D plane (Perspective)
// Returns pair(x, y)
std::pair<double, double> project_perspective(const Point_3& p) {
    // Assuming camera looks down -Z, with Y up, X right.
    // Avoid division by zero
    double z = -p.z(); 
    if (z < 0.0001) z = 0.0001; 
    
    // Simple perspective projection fov 90 (scale = 1/z)
    return { p.x() / z, p.y() / z };
}
