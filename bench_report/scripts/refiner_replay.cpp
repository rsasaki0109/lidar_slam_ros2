// Replays graph_slam_offline_runner's map refinement from a dump of its input,
// times it, and prints a digest of the result (report chapter 46).
//   refiner_replay <dump> full             : refineSubmapPoses as the runner calls it
//   refiner_replay <dump> profile [limit]  : the pyramid loop with per-step timers
// Build against the headers to test:
//   g++ -std=c++17 -O3 -DNDEBUG -I graph_based_slam/include -I /usr/include/eigen3 \
//     bench_report/scripts/refiner_replay.cpp -o refiner_replay
// The dump came from a temporary patch to runMapRefinement (not in the repository) that wrote,
// just before refineSubmapPoses: 9 doubles (cloud_downsample_voxel, prior translation sigma,
// prior rotation sigma, max step translation, max step rotation, 4 unused), 3 int64 (window
// size, window stride, submap count), each initial pose as 16 doubles (column-major), then each
// submap's local cloud as an int64 count and count x 3 floats (the clouds are float PCL points,
// so the floats hold them exactly).
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "graph_based_slam/map_refiner.hpp"

using Clock = std::chrono::steady_clock;
static double since(Clock::time_point t) {return std::chrono::duration<double>(Clock::now() - t).count();}

static std::uint64_t fnv(const void * data, std::size_t n, std::uint64_t h = 1469598103934665603ULL)
{
  const unsigned char * p = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0; i < n; ++i) {h ^= p[i]; h *= 1099511628211ULL;}
  return h;
}

int main(int argc, char ** argv)
{
  using namespace graphslam::map_refinement;
  std::ifstream in(argv[1], std::ios::binary);
  const std::string mode = argc > 2 ? argv[2] : "full";
  double d[9]; std::int64_t ints[3];
  in.read(reinterpret_cast<char *>(d), sizeof(d));
  in.read(reinterpret_cast<char *>(ints), sizeof(ints));
  MapRefinerConfig config;
  config.cloud_downsample_voxel = d[0];
  config.pyramid.window_size = static_cast<int>(ints[0]);
  config.pyramid.window_stride = static_cast<int>(ints[1]);
  for (PlaneBaConfig * ba : {&config.pyramid.window_ba, &config.pyramid.global_ba}) {
    ba->prior_translation_sigma = d[1]; ba->prior_rotation_sigma_rad = d[2];
    ba->max_step_translation = d[3]; ba->max_step_rotation_rad = d[4];
  }
  const std::size_t n = static_cast<std::size_t>(ints[2]);
  std::vector<Eigen::Matrix4d> poses(n);
  for (auto & m : poses) {in.read(reinterpret_cast<char *>(m.data()), 16 * sizeof(double));}
  std::vector<std::vector<Eigen::Vector3d>> clouds(n);
  for (auto & c : clouds) {
    std::int64_t k; in.read(reinterpret_cast<char *>(&k), sizeof(k));
    std::vector<float> xyz(static_cast<std::size_t>(k) * 3);
    in.read(reinterpret_cast<char *>(xyz.data()), xyz.size() * sizeof(float));
    c.reserve(k);
    for (std::int64_t i = 0; i < k; ++i) {c.emplace_back(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);}
  }
  std::fprintf(stderr, "loaded %zu submaps\n", n);

  if (mode == "full") {
    const auto t0 = Clock::now();
    const MapRefinerResult r = refineSubmapPoses(std::move(clouds), poses, config);
    const double sec = since(t0);
    std::uint64_t h = 1469598103934665603ULL;
    for (const auto & m : r.poses) {h = fnv(m.data(), 16 * sizeof(double), h);}
    for (const auto & line : refinerReportYamlLines(r, config)) {h = fnv(line.data(), line.size(), h);}
    std::printf("full %.3f s status=%s windows=%zu digest=%016llx\n", sec, r.status.c_str(),
      r.pyramid_result.windows.size(), static_cast<unsigned long long>(h));
    return 0;
  }

  // profile: the same steps as refineSubmapPosesImpl + refinePosesHierarchically, timed.
  const int limit = argc > 3 ? std::atoi(argv[3]) : 1 << 30;
  auto t = Clock::now();
  std::vector<std::vector<Eigen::Vector3d>> down;
  for (auto & c : clouds) {down.push_back(graphslam::map_quality::downsampleByVoxelCentroid(c, config.cloud_downsample_voxel));}
  const double t_down = since(t);
  const auto & pc = config.pyramid;
  std::vector<Eigen::Matrix4d> cur = poses;
  const std::vector<int> starts = detail::makeWindowStarts(static_cast<int>(n), pc.window_size, pc.window_stride);
  double t_world = 0, t_extract = 0, t_assoc = 0, t_ba = 0; std::size_t feats = 0; int iters = 0;
  int count = 0;
  for (int s : starts) {
    if (count++ >= limit) {break;}
    const int e = std::min<int>(static_cast<int>(n), s + pc.window_size);
    const auto wc = detail::copyCloudWindow(down, s, e - s);
    auto wp = detail::copyPoseWindow(cur, s, e - s);
    const auto wprior = detail::copyPoseWindow(poses, s, e - s);
    t = Clock::now();
    std::vector<Eigen::Vector3d> world;
    for (std::size_t p = 0; p < wc.size(); ++p) {for (const auto & q : wc[p]) {world.push_back(detail::transformPoint(wp[p], q));}}
    t_world += since(t);
    t = Clock::now();
    auto ec = pc.association.extraction; ec.collect_point_indices = true;
    const auto ex = graphslam::plane_extraction::extractPlanarPatches(world, ec);
    t_extract += since(t);
    t = Clock::now();
    const AssociationResult a = associatePlaneFeatures(wc, wp, pc.association);
    t_assoc += since(t);
    feats += a.features.size();
    if (a.features.empty()) {continue;}
    PlaneBaConfig bc = pc.window_ba; bc.fix_first_pose = true;
    t = Clock::now();
    const PlaneBaResult b = solvePlaneBa(a.features, wp, bc, wprior);
    t_ba += since(t);
    iters += b.iterations;
    for (std::size_t i = 1; i < std::min(b.poses.size(), wp.size()); ++i) {cur[s + i] = b.poses[i];}
    cur.front() = poses.front();
    (void)ex;
  }
  std::printf("profile windows=%d downsample %.2f s | per all windows: world %.2f s, extract %.2f s, "
    "associate(total, incl. its own world+extract) %.2f s, BA %.2f s (%d iterations), features %zu\n",
    count, t_down, t_world, t_extract, t_assoc, t_ba, iters, feats);
  return 0;
}
