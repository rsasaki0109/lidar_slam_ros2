// output_filter_replay (chapter 35): replay rko_lio's output position filter from the inputs a run logged with
// output_filter_log (output_filter_events.txt), with other settings. Prints one filtered position per output pose
// ('U' and 'R' events), in order. Uses rko_lio's own OutputPositionFilter.
// Build: g++ -O2 -std=c++17 -I/usr/include/eigen3 -I<rko_lio>/rko_lio/core output_filter_replay.cpp
// Usage: output_filter_replay <events> <position_sigma> <acceleration_noise> <bias_noise>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "output_position_filter.hpp"

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "usage: output_filter_replay <events> <position_sigma> <acceleration_noise> <bias_noise>\n";
    return 2;
  }
  rko_lio::core::OutputPositionFilter filter({std::atof(argv[2]), std::atof(argv[3]), std::atof(argv[4])});
  std::ifstream file(argv[1]);
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream in(line);
    char kind;
    double t;
    Eigen::Vector3d a, b;
    in >> kind >> t >> a.x() >> a.y() >> a.z();
    Eigen::Vector3d out;
    if (kind == 'P') {
      filter.propagate(t, a);
      continue;
    } else if (kind == 'U') {
      out = filter.update(t, a);
    } else {
      in >> b.x() >> b.y() >> b.z();
      filter.reset(t, a, b);
      out = a;
    }
    std::printf("%.17g %.17g %.17g %.17g\n", t, out.x(), out.y(), out.z());
  }
  return 0;
}
