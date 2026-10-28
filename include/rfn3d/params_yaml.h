#ifndef RFN3D_PARAMS_YAML_H
#define RFN3D_PARAMS_YAML_H

#include <string>

#include <rfn3d/rfn_types.h>

// Overlay the keys present in a YAML file (see config/planner.yaml) onto
// `params`; keys the file leaves out keep their current values. Unknown keys
// are reported on stderr so typos don't go unnoticed. Returns false and sets
// `error` if the file can't be read or a value has the wrong type, in which
// case `params` is left unchanged.
bool load_params_yaml(const std::string &path, planner_params_t &params, std::string &error);

#endif // RFN3D_PARAMS_YAML_H
