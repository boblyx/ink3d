#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"

bool loadConfig(const std::string& filename, Ink3d::Config& config);

void parseProjection(const std::string& input, Ink3d::Config& config);

#endif