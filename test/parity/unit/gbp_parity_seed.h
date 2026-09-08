#ifndef GBP_PARITY_SEED_H_
#define GBP_PARITY_SEED_H_

// Seed injection for planner_common/src/random_sampler.cpp.
//
// The shipped code re-seeds its std::mt19937 from std::random_device on every
// reset()/setPDF(), which makes the sampler impossible to compare across two
// builds. Neither repository is edited: the build scripts copy random_sampler.cpp
// into a scratch tree and rewrite `generator_.seed(rd())` to
// `generator_.seed(gbp_parity_seed())` there, with this header forced in via
// -include. The identical rewrite is applied to both sides, so the sampler under
// test still differs from the shipped one in exactly one respect - where the
// seed comes from - and differs in that respect identically on ROS 1 and ROS 2.

#include <cstdint>
#include <cstdlib>
#include <random>

inline std::uint32_t gbp_parity_seed() {
  const char* s = std::getenv("GBP_PARITY_SEED");
  if (s && *s) return static_cast<std::uint32_t>(std::strtoul(s, nullptr, 10));
  std::random_device rd;
  return rd();
}

#endif  // GBP_PARITY_SEED_H_
