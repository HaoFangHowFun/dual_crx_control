#pragma once
// Optional diagnostic only. One producer per Trace; flush only after it stops.
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>
#include <time.h>
#include <unistd.h>

namespace fanuc_latency
{
inline double monotonic_now() noexcept
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

class Trace
{
  struct Row
  {
    double time{};
    const char* stage{};
    std::array<double, 6> q{};
    double depth{}, age{}, alpha{}, sequence{}, controller_stamp{}, scaling{}, duration{};
  };
  std::vector<Row> rows_;
  std::string path_;
  size_t count_ = 0;

public:
  explicit Trace(const std::string& label) noexcept
  {
    const char* directory = std::getenv("FANUC_LATENCY_DIR");
    if (!directory || !*directory) return;
    try
    {
      // Allocate and touch storage before the realtime loop. Last 120 s at 1500 rows/s.
      rows_.resize(180000);
      path_ = std::string(directory) + "/" + label + "_" + std::to_string(getpid()) +
              "_" + std::to_string(static_cast<long long>(monotonic_now()*1e9)) + ".csv";
    }
    catch (...) { rows_.clear(); }
  }
  Trace(const Trace&) = delete;
  Trace& operator=(const Trace&) = delete;
  bool enabled() const noexcept { return !rows_.empty(); }

  template<class Values>
  void record(const char* stage, const Values& q, double unit_scale = 1.,
              double depth = NAN, double age = NAN, double alpha = NAN,
              double sequence = NAN, double controller_stamp = NAN,
              double scaling = NAN, double duration = NAN) noexcept
  {
    if (!enabled()) return;
    Row& row = rows_[count_++ % rows_.size()];
    row.time = monotonic_now();
    row.stage = stage;
    for (size_t i=0; i<6; ++i) row.q[i] = i < static_cast<size_t>(q.size()) ? q[i]*unit_scale : NAN;
    row.depth=depth; row.age=age; row.alpha=alpha; row.sequence=sequence;
    row.controller_stamp=controller_stamp; row.scaling=scaling; row.duration=duration;
  }

  ~Trace() noexcept
  {
    if (!enabled() || !count_) return;
    try
    {
      std::ofstream out(path_);
      if (!out) { std::fprintf(stderr, "Latency trace cannot open %s\n", path_.c_str()); return; }
      out << "time_s,stage,J1_rad,J2_rad,J3_rad,J4_rad,J5_rad,J6_rad,queue_depth,represented_age_s,alpha,controller_sequence,controller_stamp_raw,scaling,call_duration_s\n";
      out << std::setprecision(17);
      const size_t start = count_ > rows_.size() ? count_ - rows_.size() : 0;
      for (size_t i=start; i<count_; ++i)
      {
        const Row& r = rows_[i % rows_.size()];
        out << r.time << ',' << r.stage;
        for (double q : r.q) out << ',' << q;
        out << ',' << r.depth << ',' << r.age << ',' << r.alpha << ',' << r.sequence
            << ',' << r.controller_stamp << ',' << r.scaling << ',' << r.duration << '\n';
      }
      if (!out) std::fprintf(stderr, "Latency trace write failed: %s\n", path_.c_str());
      std::fprintf(stderr, "Latency trace: %s (%zu rows, %zu overwritten)\n", path_.c_str(),
                   std::min(count_, rows_.size()), start);
    }
    catch (...) { std::fprintf(stderr, "Latency trace save failed\n"); }
  }
};
}  // namespace fanuc_latency
