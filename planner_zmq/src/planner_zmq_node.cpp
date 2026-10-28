#include <blackbox/blackbox.h>
#include <rfn3d/params_yaml.h>
#include <rfn3d/planner_zmq.h>

#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>

namespace {
PlannerZmq* g_planner = nullptr;
void signal_handler(int) {
  if (g_planner) {
    g_planner->requestStop();
  }
}
}  // namespace

int main(int argc, char* argv[]) {
  // Record outputs, debug views and all console output (/log) via blackbox.
  blackbox::init("planner_zmq");

  const char* usage =
      "usage: planner_zmq [--once] [--config <planner.yaml>] [--lock-z <height_m>]\n";

  bool once = false;
  std::string config;
  std::optional<double> lock_z;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--once") == 0) {
      once = true;
    } else if (std::strcmp(argv[i], "--config") == 0) {
      if (i + 1 >= argc) {
        std::cerr << usage;
        return 1;
      }
      config = argv[++i];
    } else if (std::strcmp(argv[i], "--lock-z") == 0) {
      char* end = nullptr;
      if (i + 1 < argc) {
        lock_z = std::strtod(argv[i + 1], &end);
      }
      if (i + 1 >= argc || end == argv[i + 1] || *end != '\0') {
        std::cerr << usage;
        return 1;
      }
      ++i;
    } else {
      std::cerr << "unknown argument '" << argv[i] << "'\n" << usage;
      return 1;
    }
  }

  // An explicit --config must load; the built-in default path is optional
  // (it is absent when the binary runs outside this source tree).
  planner_params_t params;
  std::string error;
  if (!config.empty()) {
    if (!load_params_yaml(config, params, error)) {
      BLACKBOX_ERROR << error;
      return 1;
    }
    BLACKBOX_INFO << "loaded " << config;
  } else if (std::ifstream(RFN3D_DEFAULT_CONFIG).good()) {
    if (!load_params_yaml(RFN3D_DEFAULT_CONFIG, params, error)) {
      BLACKBOX_ERROR << error;
      return 1;
    }
    BLACKBOX_INFO << "loaded " << RFN3D_DEFAULT_CONFIG;
  } else {
    BLACKBOX_INFO << RFN3D_DEFAULT_CONFIG << " not found; using built-in defaults";
  }

  // --lock-z overrides the config file.
  if (lock_z) {
    params.lock_z = true;
    params.z_lock = *lock_z;
  }

  PlannerZmq planner;
  planner.setParams(params);
  if (once) {
    planner.setPlanOnce(true);
  }

  g_planner = &planner;
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  planner.run();
  blackbox::shutdown();
  return 0;
}
