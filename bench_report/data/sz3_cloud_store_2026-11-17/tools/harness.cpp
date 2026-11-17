// SZ3 stage 1 harness: reads clouds from stdin ([u64 n][x n][y n][z n][i n] float32), compresses
// each with A (graph_based_slam compressPacked), Q (step quantization + zstd), S (SZ3 EB_ABS),
// checks the round trips and prints one JSON summary.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "SZ3/api/sz.hpp"
#include "graph_based_slam/cloud_compression_core.hpp"

namespace cc = graphslam::cloud_compression::detail;
using Clock = std::chrono::steady_clock;

struct Stat {
  double xyz_bytes = 0, int_bytes = 0, total_bytes = 0, comp_s = 0, decomp_s = 0;
  double max_err[3] = {0, 0, 0};
  bool exact = true;
};

static double secs(Clock::time_point a) {
  return std::chrono::duration<double>(Clock::now() - a).count();
}

static std::vector<std::uint8_t> bytesOf(const std::vector<float> & v) {
  std::vector<std::uint8_t> b(v.size() * 4);
  std::memcpy(b.data(), v.data(), b.size());
  return b;
}

int main(int argc, char ** argv) {
  (void)argc; (void)argv;
  std::map<std::string, Stat> st;
  std::uint64_t clouds = 0, points = 0, nonfinite = 0;
  const double bounds[2] = {0.001, 0.005};
  for (;;) {
    std::uint64_t n;
    if (std::fread(&n, sizeof(n), 1, stdin) != 1) break;
    std::vector<float> ax[4];
    for (auto & a : ax) {
      a.resize(n);
      if (std::fread(a.data(), 4, n, stdin) != n) { std::cerr << "short read\n"; return 2; }
    }
    ++clouds; points += n;
    for (int k = 0; k < 3; ++k) for (float v : ax[k]) if (!std::isfinite(v)) ++nonfinite;
    // A: the current store, all four fields packed.
    {
      std::vector<std::uint8_t> packed(n * 16);
      for (std::uint64_t i = 0; i < n; ++i)
        for (int k = 0; k < 4; ++k) std::memcpy(&packed[(i * 4 + k) * 4], &ax[k][i], 4);
      auto & s = st["A"];
      std::vector<std::uint8_t> c, d;
      auto t = Clock::now();
      if (!cc::compressPacked(packed, n, 4, c)) { std::cerr << "A failed\n"; return 2; }
      s.comp_s += secs(t); t = Clock::now();
      cc::decompressPacked(c, n, 4, 16, d);
      s.decomp_s += secs(t);
      s.total_bytes += c.size();
      if (d != packed) s.exact = false;
    }
    // Intensity, stored losslessly the current way, shared by Q and S.
    std::vector<std::uint8_t> ic;
    double int_c = 0, int_d = 0;
    {
      auto ib = bytesOf(ax[3]);
      auto t = Clock::now();
      cc::compressPacked(ib, n, 1, ic);
      int_c = secs(t); t = Clock::now();
      std::vector<std::uint8_t> d;
      cc::decompressPacked(ic, n, 1, 4, d);
      int_d = secs(t);
      if (d != ib) { std::cerr << "intensity round trip\n"; return 2; }
    }
    for (int bi = 0; bi < 2; ++bi) {
      const double b = bounds[bi];
      const std::string tag = bi == 0 ? "1" : "5";
      // Q: step 2b, delta, zigzag, three uint32 fields shuffled, zstd 1.
      {
        auto & s = st["Q" + tag];
        const double step = 2 * b * 0.999;  // float32 rounding stays inside the bound
        auto t = Clock::now();
        std::vector<std::uint8_t> data(n * 12);
        for (int k = 0; k < 3; ++k) {
          std::int64_t prev = 0;
          for (std::uint64_t i = 0; i < n; ++i) {
            const std::int64_t q = std::llround(static_cast<double>(ax[k][i]) / step);
            const std::int64_t dlt = q - prev; prev = q;
            const std::uint32_t z = static_cast<std::uint32_t>((dlt << 1) ^ (dlt >> 63));
            std::memcpy(&data[(i * 3 + k) * 4], &z, 4);
          }
        }
        std::vector<std::uint8_t> c, d;
        cc::compressPacked(data, n, 3, c);
        s.comp_s += secs(t) + int_c; t = Clock::now();
        cc::decompressPacked(c, n, 3, 12, d);
        for (int k = 0; k < 3; ++k) {
          std::int64_t q = 0;
          for (std::uint64_t i = 0; i < n; ++i) {
            std::uint32_t z; std::memcpy(&z, &d[(i * 3 + k) * 4], 4);
            q += static_cast<std::int64_t>(static_cast<std::int32_t>((z >> 1) ^ (~(z & 1) + 1)));
            const float v = static_cast<float>(static_cast<double>(q) * step);
            const double e = std::fabs(static_cast<double>(v) - ax[k][i]);
            if (e > s.max_err[k]) s.max_err[k] = e;
          }
        }
        s.decomp_s += secs(t) + int_d;
        s.xyz_bytes += c.size(); s.int_bytes += ic.size(); s.total_bytes += c.size() + ic.size();
      }
      // S: SZ3, each axis a 1-D float32 array, EB_ABS b.
      {
        auto & s = st["S" + tag];
        for (int k = 0; k < 3; ++k) {
          SZ3::Config conf(n);
          conf.errorBoundMode = SZ3::EB_ABS;
          conf.absErrorBound = b;
          auto t = Clock::now();
          size_t csize = 0;
          char * c = SZ_compress<float>(conf, ax[k].data(), csize);
          s.comp_s += secs(t); t = Clock::now();
          SZ3::Config dconf;
          float * d = nullptr;
          SZ_decompress<float>(dconf, c, csize, d);
          s.decomp_s += secs(t);
          for (std::uint64_t i = 0; i < n; ++i) {
            const double e = std::fabs(static_cast<double>(d[i]) - ax[k][i]);
            if (e > s.max_err[k]) s.max_err[k] = e;
          }
          delete[] c; delete[] d;
          s.xyz_bytes += csize; s.total_bytes += csize;
        }
        s.comp_s += int_c; s.decomp_s += int_d;
        s.int_bytes += ic.size(); s.total_bytes += ic.size();
      }
    }
  }
  std::printf("{\"clouds\": %llu, \"points\": %llu, \"nonfinite_xyz\": %llu, \"raw_bytes\": %llu,"
    " \"methods\": {", (unsigned long long)clouds, (unsigned long long)points,
    (unsigned long long)nonfinite, (unsigned long long)(points * 16));
  bool first = true;
  for (auto & [k, s] : st) {
    std::printf("%s\"%s\": {\"total_bytes\": %.0f, \"xyz_bytes\": %.0f, \"int_bytes\": %.0f,"
      " \"comp_s\": %.4f, \"decomp_s\": %.4f, \"max_err\": [%.7f, %.7f, %.7f], \"exact\": %s}",
      first ? "" : ", ", k.c_str(), s.total_bytes, s.xyz_bytes, s.int_bytes, s.comp_s,
      s.decomp_s, s.max_err[0], s.max_err[1], s.max_err[2], s.exact ? "true" : "false");
    first = false;
  }
  std::printf("}}\n");
  return 0;
}
