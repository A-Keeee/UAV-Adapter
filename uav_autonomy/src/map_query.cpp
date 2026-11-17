#include <octomap/OcTree.h>

#include <iostream>

// Offline artifact inspection only. Query coordinates come from the isolated
// evaluator; this executable publishes no ROS data and cannot command a vehicle.
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  octomap::OcTree map(.2);
  if (!map.readBinary(argv[1])) return 3;
  double x, y, z;
  while (std::cin >> x >> y >> z) {
    auto node = map.search(x, y, z);
    std::cout << (node ? (map.isNodeOccupied(node) ? 1 : 0) : -1) << '\n';
  }
  return std::cin.eof() ? 0 : 4;
}
