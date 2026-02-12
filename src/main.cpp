/**
 * ink3d
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>
#include <variant>
#include <optional>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3                              Point_3;
typedef K::Vector_3                             Vector_3;
typedef K::Ray_3                                Ray_3;
typedef K::Segment_3                            Segment_3;
typedef K::FT                                   FT;

typedef CGAL::Surface_mesh<Point_3>             Mesh;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Mesh>::edge_descriptor edge_descriptor;

// AABB Tree types for visibility checks
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
typedef CGAL::AABB_traits_3<K, Primitive>              Traits;
typedef CGAL::AABB_tree<Traits>                        Tree;
typedef Tree::Intersection_and_primitive_id<Ray_3>::Type Ray_intersection;

typedef Tree::Intersection_and_primitive_id<Segment_3>::Type Seg_intersection;

// Helper: Construct a LookAt matrix (World -> Camera)
// Eye at (1,1,1), looking at (0,0,0), Up is roughly Z
CGAL::Aff_transformation_3<K> look_at_transform(const Point_3& eye, const Point_3& center, const Vector_3& up) {
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

static inline double sqr(double x) { return x * x; }

bool is_edge_relevant(edge_descriptor ed, const Mesh& mesh, const Point_3& eye) {
    halfedge_descriptor h = halfedge(ed, mesh);
    face_descriptor f1 = face(h, mesh);
    face_descriptor f2 = face(opposite(h, mesh), mesh);

    auto is_back_facing = [&](face_descriptor f) {
        if (f == Mesh::null_face()) return true;
        auto n = CGAL::Polygon_mesh_processing::compute_face_normal(f, mesh);
        // Vector from eye to a point on the face
        Vector_3 view_dir = mesh.point(CGAL::target(h, mesh)) - eye;
        return (n * view_dir) >= 0; // Normal points away from eye
    };

    bool b1 = is_back_facing(f1);
    bool b2 = is_back_facing(f2);

    // Only render if at least one face is front-facing
    // To only render SILHOUETTES + CREASES, you'd add an angle check here.
    return !(b1 && b2);
}

bool is_point_visible_from_eye(
    const Point_3& eye,
    const Point_3& p,
    const Tree& tree,
    const std::array<face_descriptor, 2>& ignore_faces,
    const double eps_rel = 1e-7,
    const double eps_abs = 1e-12
) {
    Vector_3 v = p - eye;
    double d2 = CGAL::to_double(v.squared_length());
    if (d2 < eps_abs) return true;

    double d = std::sqrt(d2);
    double eps = std::max(eps_abs, eps_rel * d);
    double eps2 = eps * eps;

    // Shorten the segment slightly to avoid hitting the target point itself
    Point_3 p_shorten = eye + v * ((d - eps) / d);
    Segment_3 seg(eye, p_shorten);

    std::vector<Seg_intersection> hits;
    tree.all_intersections(seg, std::back_inserter(hits));

    for (const auto& inter : hits) {
        // 1. Filter ignored faces (like the face the eye/point lies on)
        face_descriptor f = inter.second;
        if (f == ignore_faces[0] || f == ignore_faces[1]) continue;

        Point_3 intersection_point;
        
        // 2. Use std::get_if to safely inspect the variant
        if (const Point_3* pt = std::get_if<Point_3>(&inter.first)) {
            intersection_point = *pt;
        } 
        else if (const Segment_3* s = std::get_if<Segment_3>(&inter.first)) {
            // If the intersection is a segment, take the point closest to the eye
            double a2 = CGAL::to_double((s->source() - eye).squared_length());
            double b2 = CGAL::to_double((s->target() - eye).squared_length());
            intersection_point = (a2 < b2) ? s->source() : s->target();
        } 
        else {
            continue; 
        }

        // 3. Final check: is this intersection point far enough from the eye to be an obstacle?
        double t2 = CGAL::to_double((intersection_point - eye).squared_length());
        if (t2 > eps2) {
            return false; // Found a valid occluder
        }
    }
    return true; // No valid occluders found
}

bool is_point_visible_robust(
    const Point_3& eye, const Point_3& p, 
    const Tree& tree, const Mesh& mesh, 
    const std::array<face_descriptor, 2>& ignore_faces
) {
    // Distance from eye to point
    double dist = std::sqrt(CGAL::to_double((p - eye).squared_length()));
    
    // Move target slightly toward eye to avoid self-intersection with the target vertex
    Vector_3 dir = eye - p;
    Point_3 p_offset = p + dir * (1e-6 / dist); 

    Segment_3 seg(eye, p_offset);
    
    // Check all intersections
    std::vector<Seg_intersection> hits;
    tree.all_intersections(seg, std::back_inserter(hits));

    for (const auto& inter : hits) {
        face_descriptor f = inter.second;
        if (f == ignore_faces[0] || f == ignore_faces[1]) continue;

        // If we hit anything else, the point is occluded
        return false;
    }
    return true;
}

std::vector<std::pair<Point_3, Point_3>> visible_subsegments_world(
    const Point_3& eye,
    const Point_3& a,
    const Point_3& b,
    const Tree& tree,
    const std::array<face_descriptor,2>& ignore_faces,
    int samples = 42,
    int refine_steps = 32
){
    std::vector<std::pair<Point_3, Point_3>> out;
    if (samples < 2) samples = 2;

    auto point_on = [&](double t)->Point_3 {
        return Point_3(
            a.x() + (b.x() - a.x()) * t,
            a.y() + (b.y() - a.y()) * t,
            a.z() + (b.z() - a.z()) * t
        );
    };

    std::vector<double> ts(samples);
    std::vector<char> vis(samples);

    for (int i = 0; i < samples; ++i) {
        double t = double(i) / double(samples - 1);
        ts[i] = t;
        Point_3 p = point_on(t);
        vis[i] = is_point_visible_from_eye(eye, p, tree, ignore_faces) ? 1 : 0;
    }

    // If everything hidden or visible, handle quickly
    bool any_vis = false, any_hid = false;
    for (char v : vis) { any_vis |= (v==1); any_hid |= (v==0); }
    if (!any_vis) return out;
    if (!any_hid) { out.emplace_back(a, b); return out; }

    // Helper: find boundary t where visibility flips using bisection
    auto refine_boundary = [&](double t0, double t1, bool want_visible_at_t1)->double {
        double lo = t0, hi = t1;
        for (int k = 0; k < refine_steps; ++k) {
            double mid = 0.5 * (lo + hi);
            bool mvis = is_point_visible_from_eye(eye, point_on(mid), tree, ignore_faces);
            if (mvis == want_visible_at_t1) hi = mid;
            else lo = mid;
        }
        return 0.5 * (lo + hi);
    };

    // Walk through samples and build visible ranges [t_start, t_end]
    int i = 0;
    while (i < samples - 1) {
        // find next visible
        while (i < samples && vis[i] == 0) i++;
        if (i >= samples) break;

        double t_start = ts[i];

        // if visibility starts in middle, refine boundary with previous sample
        if (i > 0 && vis[i-1] == 0) {
            t_start = refine_boundary(ts[i-1], ts[i], true);
        }

        // advance until hidden
        int j = i;
        while (j < samples && vis[j] == 1) j++;
        double t_end = ts[j-1];

        // if it ends before last, refine boundary with next hidden sample
        if (j < samples && vis[j] == 0) {
            t_end = refine_boundary(ts[j-1], ts[j], false);
        }

        // Emit subsegment
        if (t_end > t_start) {
            out.emplace_back(point_on(t_start), point_on(t_end));
        }

        i = j;
    }

    return out;
}

bool is_feature_edge(edge_descriptor ed, const Mesh& mesh, const Point_3& eye, double angle_threshold_deg = 20.0) {
    halfedge_descriptor hd = halfedge(ed, mesh);
    face_descriptor f1 = face(hd, mesh);
    face_descriptor f2 = face(opposite(hd, mesh), mesh);

    // 1. Always keep boundary edges (edges with only one face)
    if (f1 == Mesh::null_face() || f2 == Mesh::null_face()) return true;

    // 2. Compute Normals
    Vector_3 n1 = CGAL::Polygon_mesh_processing::compute_face_normal(f1, mesh);
    Vector_3 n2 = CGAL::Polygon_mesh_processing::compute_face_normal(f2, mesh);

    // 3. Silhouette Check: Is one face pointing toward eye and the other away?
    Vector_3 view_dir = mesh.point(CGAL::target(hd, mesh)) - eye;
    double dot1 = CGAL::to_double(n1 * view_dir);
    double dot2 = CGAL::to_double(n2 * view_dir);
    
    // If signs are different, it's a silhouette edge
    if ((dot1 > 0) != (dot2 > 0)) return true;

    // 4. Back-face Culling: If both faces point away, hide it
    if (dot1 > 0 && dot2 > 0) return false;

    // 5. Crease Check: Dihedral angle
    // cos(theta) = (n1 . n2) / (|n1||n2|)
    double cos_theta = CGAL::to_double((n1 * n2) / std::sqrt(n1.squared_length() * n2.squared_length()));
    
    // Clamp for safety
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    double angle = std::acos(cos_theta) * 180.0 / M_PI;

    return (angle > angle_threshold_deg);
}


int main(int argc, char* argv[]) {
    const std::string filename = (argc > 1) ? argv[1] : "input.off";
    const std::string svg_filename = "output.svg";

    Mesh mesh;
    if(!CGAL::IO::read_polygon_mesh(filename, mesh)) {
        std::cerr << "Error: Cannot read mesh " << filename << std::endl;
        return 1;
    }

    // 1. Setup Camera
    Point_3 eye(1, 1, 1);
    Point_3 center(0, 0, 0);
    Vector_3 up(0, 0, 1); // Z-up world
    
    // View transform: World -> Camera
    auto view_matrix = look_at_transform(eye, center, up);

    // 2. Build AABB Tree for Visibility Testing
    // We test visibility in World Space to avoid transforming the whole tree
    Tree tree(faces(mesh).first, faces(mesh).second, mesh);
    tree.build();

    std::vector<std::pair<Point_3, Point_3>> visible_segments_camera_space;

    // 3. Iterate edges and check visibility
    std::cout << "Processing edges for HLR..." << std::endl;
    
    for(edge_descriptor ed : edges(mesh)) {
        if (!is_edge_relevant(ed, mesh, eye)) continue;
        if (!is_feature_edge(ed, mesh, eye, 20.0)) continue;
        halfedge_descriptor hd = halfedge(ed, mesh);

        Point_3 source = mesh.point(CGAL::source(hd, mesh));
        Point_3 target = mesh.point(CGAL::target(hd, mesh));

        // faces incident to this edge (may be null_face on boundary)
        face_descriptor f1 = face(hd, mesh);
        face_descriptor f2 = face(opposite(hd, mesh), mesh);

        std::array<face_descriptor,2> ignore_faces = { f1, f2 };

        // compute visible parts in WORLD space
        auto parts = visible_subsegments_world(
            eye, source, target, tree, ignore_faces,
            /*samples*/ 108, /*refine_steps*/ 108
        );

        for (auto& wseg : parts) {
            // Transform to camera space
            Point_3 p1_cam = view_matrix.transform(wseg.first);
            Point_3 p2_cam = view_matrix.transform(wseg.second);

            // basic near-plane reject (you probably want proper clipping, but keep your check)
            if (p1_cam.z() > -0.1 || p2_cam.z() > -0.1) continue;

            visible_segments_camera_space.emplace_back(p1_cam, p2_cam);
        }
    }
    // 4. Project and Compute SVG Bounds
    if (visible_segments_camera_space.empty()) {
        std::cerr << "No visible edges found. Check camera direction." << std::endl;
        return 0;
    }

    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> lines_2d;

    for (const auto& seg : visible_segments_camera_space) {
        auto uv1 = project_perspective(seg.first);
        auto uv2 = project_perspective(seg.second);

        // Flip Y for SVG (SVG coords: Y down, Camera coords: Y up)
        uv1.second = -uv1.second;
        uv2.second = -uv2.second;

        lines_2d.push_back({uv1, uv2});

        min_x = std::min({min_x, uv1.first, uv2.first});
        max_x = std::max({max_x, uv1.first, uv2.first});
        min_y = std::min({min_y, uv1.second, uv2.second});
        max_y = std::max({max_y, uv1.second, uv2.second});
    }

    // Add some padding
    double width = max_x - min_x;
    double height = max_y - min_y;
    double padding = std::max(width, height) * 0.1;
    min_x -= padding; min_y -= padding;
    width += 2*padding; height += 2*padding;

    // 5. Write SVG
    std::ofstream svg(svg_filename);
    svg << "<svg xmlns='http://www.w3.org/2000/svg' viewBox='" 
        << min_x << " " << min_y << " " << width << " " << height << "' "
        << "width='800' height='800' style='background-color:white'>\n";
    
    // Style
    svg << "<style> line { stroke: black; stroke-width: " << 1.0 << "; vector-effect: non-scaling-stroke; } </style>\n";

    for (const auto& line : lines_2d) {
        svg << "<line x1='" << line.first.first << "' y1='" << line.first.second 
            << "' x2='" << line.second.first << "' y2='" << line.second.second << "' />\n";
    }

    svg << "</svg>";
    std::cout << "Successfully wrote " << lines_2d.size() << " visible edges to " << svg_filename << std::endl;

    return 0;
}
