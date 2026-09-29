#include <zmq.hpp>

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char* argv[]) {
  zmq::context_t ctx(1);
  zmq::socket_t push(ctx, zmq::socket_type::pub);

  std::string endpoint = "ipc:///tmp/rfn3d_goal";
  if (argc >= 2) {
    endpoint = argv[1];
  }

  push.connect(endpoint);

  std::cout << "rfn3d goal sender (connected to " << endpoint << ")\n";
  std::cout << "Enter goal as: x y z   (or 'q' to quit)\n\n";

  std::string line;
  while (true) {
    std::cout << "goal> ";
    if (!std::getline(std::cin, line)) {
      break;
    }

    if (line.empty()) {
      continue;
    }
    if (line == "q" || line == "quit" || line == "exit") {
      break;
    }

    std::istringstream iss(line);
    double x, y, z;
    if (!(iss >> x >> y >> z)) {
      std::cerr << "  invalid input, expected: x y z\n";
      continue;
    }

    double goal[3] = {x, y, z};
    push.send(zmq::buffer(goal, sizeof(goal)), zmq::send_flags::none);
    std::cout << "  sent goal: " << x << " " << y << " " << z << "\n";
  }

  return 0;
}
