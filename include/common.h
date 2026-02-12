#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

namespace Ink3d {

    // Kernel Alias
    using K = CGAL::Exact_predicates_inexact_constructions_kernel;

    // Basic Geometric Types
    using Point_3   = K::Point_3;
    using Vector_3  = K::Vector_3;
    using Ray_3     = K::Ray_3;
    using Segment_3 = K::Segment_3;
    using FT        = K::FT;

    // Mesh Types
    using Mesh                = CGAL::Surface_mesh<Point_3>;
    using face_descriptor     = boost::graph_traits<Mesh>::face_descriptor;
    using halfedge_descriptor = boost::graph_traits<Mesh>::halfedge_descriptor;
    using edge_descriptor     = boost::graph_traits<Mesh>::edge_descriptor;

    // AABB Tree types for visibility checks
    using Primitive        = CGAL::AABB_face_graph_triangle_primitive<Mesh>;
    using Traits           = CGAL::AABB_traits_3<K, Primitive>;
    using Tree             = CGAL::AABB_tree<Traits>;
    using Ray_intersection = Tree::Intersection_and_primitive_id<Ray_3>::Type;
    using Seg_intersection = Tree::Intersection_and_primitive_id<Segment_3>::Type;

} // namespace Ink3d
#endif