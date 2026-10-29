#ifndef RFN3D_PARAMS_YAML_H
#define RFN3D_PARAMS_YAML_H

#include <string>

#include <rfn3d/rfn_types.h>

bool load_params_yaml(const std::string &path, planner_params_t &params, std::string &error);

#endif // RFN3D_PARAMS_YAML_H
