#ifndef PROJECTION_H
#define PROJECTION_H

#include "common.h"

CGAL::Aff_transformation_3<Ink3d::K> look_at_transform(const Ink3d::Point_3& eye, const Ink3d::Point_3& center, const Ink3d::Vector_3& up);

std::pair<double, double> project_perspective(const Ink3d::Point_3& p);
#endif