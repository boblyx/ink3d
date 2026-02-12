#ifndef VISIBILITY_H
#define VISIBLITY_H

#include "common.h"

bool is_point_visible_from_eye(
    const Ink3d::Point_3& eye,
    const Ink3d::Point_3& p,
    const Ink3d::Tree& tree,
    const std::array<Ink3d::face_descriptor, 2>& ignore_faces,
    const double eps_rel = 1e-7,
    const double eps_abs = 1e-12
);

bool is_point_visible_robust(
    const Ink3d::Point_3& eye, const Ink3d::Point_3& p, 
    const Ink3d::Tree& tree, const Ink3d::Mesh& mesh, 
    const std::array<Ink3d::face_descriptor, 2>& ignore_faces
);

std::vector<std::pair<Ink3d::Point_3, Ink3d::Point_3>> visible_subsegments_world(
    const Ink3d::Point_3& eye,
    const Ink3d::Point_3& a,
    const Ink3d::Point_3& b,
    const Ink3d::Tree& tree,
    const std::array<Ink3d::face_descriptor,2>& ignore_faces,
    int samples = 42,
    int refine_steps = 32
);

bool is_feature_edge(Ink3d::edge_descriptor ed, const Ink3d::Mesh& mesh, const Ink3d::Point_3& eye, double angle_threshold_deg = 20.0);

bool is_edge_relevant(Ink3d::edge_descriptor ed, const Ink3d::Mesh& mesh, const Ink3d::Point_3& eye);

#endif