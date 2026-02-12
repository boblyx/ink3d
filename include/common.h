#ifndef COMMON_H
#define COMMON_H

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

    enum class ProjectionType {
        Orthographic,
        Perspective,
        Unknown // Useful for default initialization
    };

    inline const Point_3 DEFAULT_EYE (1, 1, 1);
    inline const Point_3 DEFAULT_CENTER(0, 0, 0);
    inline const Vector_3 DEFAULT_UP(0, 0, 1); // Z-up world

    struct Config {
        std::string input_path="input.ply";
        std::string output_path="output.svg";
        K::Point_3 view_origin = DEFAULT_EYE;
        K::Point_3 look_at = DEFAULT_CENTER;
        ProjectionType projection = ProjectionType::Perspective; // Default value
        int samples=108;
        int refine_steps=108;
    };

} // namespace Ink3d
#endif