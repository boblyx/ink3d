#ifndef CONFIG_STRUCTS_H
#define CONFIG_STRUCTS_H

namespace Ink3d{

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
}
#endif