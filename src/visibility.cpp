/**
 * visiblity.cpp
 */

#include <vector>
#include <cmath>
#include <limits>
#include <variant>
#include <optional>

#include "common.h"
using namespace Ink3d;

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
