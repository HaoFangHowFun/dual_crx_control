#include "../diagnostics/latency_trace.hpp"
#include <cassert>
int main(int argc, char** argv)
{
  assert(argc == 2);
  unsetenv("FANUC_LATENCY_DIR");
  { fanuc_latency::Trace disabled("disabled"); assert(!disabled.enabled()); }
  setenv("FANUC_LATENCY_DIR", argv[1], 1);
  fanuc_latency::Trace trace("smoke");
  assert(trace.enabled());
  std::array<double, 6> q{};
  for (int i=0; i<180003; ++i)
  {
    q[0] = i;
    trace.record("test", q, 1., 8., .016, .5, i, i*2, 1.);
  }
}
