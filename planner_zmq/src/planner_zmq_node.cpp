#include <rfn3d/planner_zmq.h>

#include <csignal>
#include <cstring>

namespace {
PlannerZmq* g_planner = nullptr;
void signal_handler(int) {
  if (g_planner) {
    g_planner->requestStop();
  }
}
}  // namespace

int main(int argc, char* argv[]) {
  bool once = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--once") == 0) {
      once = true;
    }
  }

  PlannerZmq planner;
  if (once) {
    planner.setPlanOnce(true);
  }

  g_planner = &planner;
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  planner.run();
  return 0;
}
