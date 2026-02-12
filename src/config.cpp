#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept> // For error handling

#include <json/json.h>

#include "common.h"

// Logic snippet for your parser
void parseProjection(const std::string& input, Ink3d::Config& config) {
    if (input == "orthographic") {
        config.projection = Ink3d::ProjectionType::Orthographic;
        return;
    } 
    config.projection = Ink3d::ProjectionType::Perspective;
}

bool loadConfig(const std::string& filename, Ink3d::Config& config) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;

    if (!Json::parseFromStream(builder, file, &root, &errs)) return false;

    // 1. Parse Vectors
    auto parseVec = [](const Json::Value& node) {
        std::vector<double> res;
        for (const auto& val : node) res.push_back(val.asDouble());
        Ink3d::Point_3 p(res[0], res[1], res[2]);
        return p;
    };


    config.view_origin = parseVec(root["view_origin"]);
    config.look_at     = (parseVec(root["look_at"]));

    // 2. Validate and Map Projection String
    std::string projStr = root["projection"].asString();
    std::string input_path = root["input_path"].asString();
    std::string output_path = root["output_path"].asString();
    int samples = root["samples"].asInt();
    int refine_steps = root["refine_steps"].asInt();

    config.samples = samples;
    config.refine_steps = refine_steps;
    config.input_path = input_path;
    config.output_path = output_path;

    if (projStr == "orthographic") {
        config.projection = Ink3d::ProjectionType::Orthographic;
    } 
    else if (projStr == "perspective") {
        config.projection = Ink3d::ProjectionType::Perspective;
    } 
    else {
        // Validation failed
        std::cerr << "Error: Invalid projection type '" << projStr 
                  << "'. Must be 'orthographic' or 'perspective'." << std::endl;
        return false;
    }

    return true;
}