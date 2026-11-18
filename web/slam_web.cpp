// RKO-LIO odometry for the browser demo: rko_lio::driver::OfflineDriver behind a small C API.
// The page (docs/demo) feeds it IMU samples and scans decoded from an LSWC2 clip
// (web/make_clip.py). Built natively, main() replays an uncompressed clip and writes a TUM
// trajectory, so a WebAssembly run can be compared with a native one (web/README.md).
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "rko_lio/driver/driver_config.hpp"
#include "rko_lio/driver/offline_driver.hpp"
#include "rko_lio/driver/parameters.hpp"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define API extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define API extern "C"
#endif

namespace {

class JsonParameters : public rko_lio::driver::Parameters {
public:
  explicit JsonParameters(nlohmann::json values) : values_(std::move(values)) {}
  bool get_bool(const std::string& n, bool d) override { return get(n, d); }
  int64_t get_int(const std::string& n, int64_t d) override { return get(n, d); }
  double get_double(const std::string& n, double d) override { return get(n, d); }
  std::string get_string(const std::string& n, const std::string& d) override { return get(n, d); }
  std::vector<double> get_double_array(const std::string& n, const std::vector<double>& d) override {
    return get(n, d);
  }
  std::vector<int64_t> get_int_array(const std::string& n, const std::vector<int64_t>& d) override {
    return get(n, d);
  }
  std::string get_required_string(const std::string& n) override {
    if (!values_.contains(n)) throw std::runtime_error("missing parameter " + n);
    return values_.at(n).get<std::string>();
  }
  void info(const std::string& m) override { std::cerr << "[rko_lio] [INFO] " << m << "\n"; }
  void warn(const std::string& m) override { std::cerr << "[rko_lio] [WARN] " << m << "\n"; }

private:
  template <typename T>
  T get(const std::string& n, const T& d) {
    return values_.contains(n) && !values_.at(n).is_null() ? values_.at(n).get<T>() : d;
  }
  nlohmann::json values_;
};

struct Output {
  double time_s;
  double pose[7];  // x y z qx qy qz qw
  std::vector<float> world_points;  // deskewed scan in the odometry frame
};

class Session {
public:
  explicit Session(const std::string& params_json) {
    JsonParameters params(nlohmann::json::parse(params_json));
    auto config = rko_lio::driver::declare_driver_config(params);
    driver_ = std::make_unique<rko_lio::driver::OfflineDriver>(
        std::move(config), [this](const rko_lio::driver::OdometryOutput& o) { on_output(o); },
        [](const std::string& m) { std::cerr << "[rko_lio] [INFO] " << m << "\n"; },
        [](const std::string& m) { std::cerr << "[rko_lio] [WARN] " << m << "\n"; }, 10);
  }
  void add_imu(int32_t sec, uint32_t nsec, const double* v) {
    driver_->add_imu(rko_lio::core::Nsec(int64_t(sec) * 1000000000LL + nsec), Eigen::Vector3d(v[0], v[1], v[2]),
                     Eigen::Vector3d(v[3], v[4], v[5]));
  }
  // xyz: n x int16[3] in centimetres; dt: n x uint16 point times after the header stamp in
  // 2-microsecond units. The cloud handed to the driver has float32 x, y, z and a float64
  // absolute "timestamp" in nanoseconds, as a Livox MID-360 driver publishes it.
  void add_scan(int32_t sec, uint32_t nsec, uint32_t n, const int16_t* xyz, const uint16_t* dt) {
    const int64_t header_ns = int64_t(sec) * 1000000000LL + nsec;
    buffer_.resize(std::size_t(n) * 20);
    for (uint32_t i = 0; i < n; ++i) {
      float p[3] = {xyz[3 * i] * 0.01f, xyz[3 * i + 1] * 0.01f, xyz[3 * i + 2] * 0.01f};
      const double ts = double(header_ns) + double(dt[i]) * 2000.0;
      std::memcpy(&buffer_[std::size_t(i) * 20], p, 12);
      std::memcpy(&buffer_[std::size_t(i) * 20 + 12], &ts, 8);
    }
    rko_lio::driver::PointCloudView view;
    view.height = 1;
    view.width = n;
    view.point_step = 20;
    view.fields = {{"x", 0, 7, 1}, {"y", 4, 7, 1}, {"z", 8, 7, 1}, {"timestamp", 12, 8, 1}};
    view.data = buffer_.data();
    view.data_size = buffer_.size();
    driver_->add_lidar(view, rko_lio::core::Nsec(header_ns));
  }
  std::vector<Output> outputs;

private:
  void on_output(const rko_lio::driver::OdometryOutput& o) {
    const auto& t = o.pose.translation();
    const auto& q = o.pose.unit_quaternion();
    Output out{o.time.count() * 1e-9, {t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()}, {}};
    out.world_points.reserve(o.deskewed_scan.size() * 3);
    for (const auto& p : o.deskewed_scan) {
      const Eigen::Vector3d w = o.pose * p;
      out.world_points.insert(out.world_points.end(), {float(w.x()), float(w.y()), float(w.z())});
    }
    outputs.push_back(std::move(out));
  }
  std::unique_ptr<rko_lio::driver::OfflineDriver> driver_;
  std::vector<uint8_t> buffer_;
};

std::unique_ptr<Session> g_session;

}  // namespace

API int slam_create(const char* params_json) {
  try {
    g_session = std::make_unique<Session>(params_json);
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "slam_create: " << e.what() << "\n";
    return 0;
  }
}
API void slam_add_imu(int32_t sec, uint32_t nsec, const double* values) { g_session->add_imu(sec, nsec, values); }
API void slam_add_scan(int32_t sec, uint32_t nsec, uint32_t n, const int16_t* xyz, const uint16_t* dt) {
  g_session->add_scan(sec, nsec, n, xyz, dt);
}
API int slam_output_count() { return int(g_session->outputs.size()); }
API const double* slam_output_pose(int i) { return &g_session->outputs[i].time_s; }
API int slam_output_point_count(int i) { return int(g_session->outputs[i].world_points.size() / 3); }
API const float* slam_output_points(int i) { return g_session->outputs[i].world_points.data(); }
API void slam_clear_outputs() { g_session->outputs.clear(); }

#ifndef SLAM_WEB_NO_MAIN
namespace {
// Decodes n zigzag-delta uint16 values into out[i * stride].
template <typename T>
void zigzag_delta_decode(const uint16_t* src, uint32_t n, T* out, int stride) {
  int32_t value = 0;
  for (uint32_t i = 0; i < n; ++i) {
    const uint32_t z = src[i];
    value += int32_t(z >> 1) ^ -int32_t(z & 1);
    out[std::size_t(i) * stride] = T(value);
  }
}
}  // namespace

// slam_web clip.lswc2 params.json out.tum (the clip uncompressed: gunzip -k clip.lswc2.gz)
int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: slam_web clip.lswc2 params.json out.tum\n";
    return 2;
  }
  std::ifstream in(argv[1], std::ios::binary);
  std::vector<char> blob((std::istreambuf_iterator<char>(in)), {});
  if (blob.size() < 12 || std::string(blob.data(), 5) != "LSWC2") {
    std::cerr << "not an uncompressed LSWC2 clip\n";
    return 2;
  }
  std::ifstream pin(argv[2]);
  std::string params((std::istreambuf_iterator<char>(pin)), {});
  if (!slam_create(params.c_str())) return 1;
  std::size_t off = 8;
  uint32_t json_len;
  std::memcpy(&json_len, &blob[off], 4);
  off += 4;
  const auto header = nlohmann::json::parse(std::string(&blob[off], json_len));
  off += json_len;
  const std::size_t n_imu = header.at("n_imu"), n_scans = header.at("n_scans");
  struct Imu {
    int32_t sec;
    uint32_t nsec;
    double v[6];
  };
  std::vector<Imu> imus(n_imu);
  for (auto& m : imus) {
    std::memcpy(&m.sec, &blob[off], 4);
    std::memcpy(&m.nsec, &blob[off + 4], 4);
    std::memcpy(m.v, &blob[off + 8], 48);
    off += 56;
  }
  std::size_t next_imu = 0;
  auto stamp = [](int32_t s, uint32_t ns) { return int64_t(s) * 1000000000LL + ns; };
  std::vector<int16_t> xyz;
  std::vector<uint16_t> dt;
  for (std::size_t k = 0; k < n_scans; ++k) {
    int32_t sec;
    uint32_t nsec, n;
    std::memcpy(&sec, &blob[off], 4);
    std::memcpy(&nsec, &blob[off + 4], 4);
    std::memcpy(&n, &blob[off + 8], 4);
    off += 12;
    xyz.resize(std::size_t(n) * 3);
    dt.resize(n);
    std::vector<uint16_t> column(n);
    for (int c = 0; c < 4; ++c) {
      std::memcpy(column.data(), &blob[off], std::size_t(n) * 2);
      off += std::size_t(n) * 2;
      if (c < 3) {
        zigzag_delta_decode(column.data(), n, xyz.data() + c, 3);
      } else {
        zigzag_delta_decode(column.data(), n, dt.data(), 1);
      }
    }
    while (next_imu < n_imu && stamp(imus[next_imu].sec, imus[next_imu].nsec) <= stamp(sec, nsec)) {
      slam_add_imu(imus[next_imu].sec, imus[next_imu].nsec, imus[next_imu].v);
      ++next_imu;
    }
    slam_add_scan(sec, nsec, n, xyz.data(), dt.data());
  }
  while (next_imu < n_imu) {
    slam_add_imu(imus[next_imu].sec, imus[next_imu].nsec, imus[next_imu].v);
    ++next_imu;
  }
  std::ofstream out(argv[3]);
  out.precision(9);
  for (int i = 0; i < slam_output_count(); ++i) {
    const double* p = slam_output_pose(i);
    out << std::fixed << p[0];
    for (int j = 1; j < 8; ++j) out << ' ' << p[j];
    out << '\n';
  }
  std::cerr << "poses " << slam_output_count() << "\n";
  return 0;
}
#endif
