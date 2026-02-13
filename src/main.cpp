/**
 * main.cpp
 */

#include "common.h"
#include "visibility.h"
#include "projection.h"
#include "config.h"

using namespace Ink3d;

int main(int argc, char* argv[]) {
    Config cnf; 
    if(loadConfig(argv[1], cnf)){
        std::cout << cnf.input_path << std::endl;
    } else {
        std::cout << "Failed to load config" << std::endl;
    }
    const std::string filename = cnf.input_path;
    const std::string svg_filename = cnf.output_path;

    Mesh mesh;
    if(!CGAL::IO::read_polygon_mesh(filename, mesh)) {
        std::cerr << "Error: Cannot read mesh " << filename << std::endl;
        return 1;
    }

    // 1. Setup Camera
    Point_3 eye = cnf.view_origin;
    Point_3 center = cnf.look_at;
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
            cnf.projection,
            eye, center,
            source, target, tree, ignore_faces,
            cnf.samples , 
            cnf.refine_steps
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
        std::pair<double, double> uv1;
        std::pair<double, double> uv2;
        if (cnf.projection == ProjectionType::Orthographic){
            uv1 = {seg.first.x(), seg.first.y()};
            uv2 = {seg.second.x(), seg.second.y()};
        }else{
            uv1 = project_perspective(seg.first);
            uv2 = project_perspective(seg.second);
        }

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
