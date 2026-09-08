#ifndef GBP_PARITY_EMIT_H_
#define GBP_PARITY_EMIT_H_

// Machine-readable result channel shared by both sides of the parity suite.
//
// One `key = value` per line into the file named by GBP_PARITY_OUT (stdout if
// unset). Doubles go out as %.17g because that is
// the shortest form guaranteed to round-trip an IEEE-754 double, so the
// comparator sees exactly the bits the binary computed and a "difference" is
// never an artefact of the printing.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace parity {

// ROS 1's rosconsole writes INFO and DEBUG to stdout, so the results cannot
// share it. GBP_PARITY_OUT names a file that carries nothing but results.
inline std::FILE* out() {
  static std::FILE* f = [] {
    const char* path = std::getenv("GBP_PARITY_OUT");
    if (!path || !*path) return stdout;
    std::FILE* h = std::fopen(path, "w");
    if (!h) {
      std::fprintf(stderr, "parity: cannot open %s\n", path);
      std::abort();
    }
    return h;
  }();
  return f;
}

// Called between the risky probes in the RRG section so that a crash there
// still leaves everything emitted so far on disk.
inline void flush() { std::fflush(out()); }

inline void emit_raw(const std::string& key, const std::string& value) {
  std::fprintf(out(), "%s = %s\n", key.c_str(), value.c_str());
}

inline void emit_i(const std::string& key, long long v) {
  std::fprintf(out(), "%s = %lld\n", key.c_str(), v);
}

inline void emit_b(const std::string& key, bool v) {
  std::fprintf(out(), "%s = %d\n", key.c_str(), v ? 1 : 0);
}

inline void emit_d(const std::string& key, double v) {
  std::fprintf(out(), "%s = %.17g\n", key.c_str(), v);
}

inline void emit_s(const std::string& key, const std::string& v) {
  std::fprintf(out(), "%s = \"%s\"\n", key.c_str(), v.c_str());
}

inline void emit_darray(const std::string& key, const std::vector<double>& v) {
  std::string out = "[";
  char buf[40];
  for (size_t i = 0; i < v.size(); ++i) {
    std::snprintf(buf, sizeof(buf), "%.17g", v[i]);
    if (i) out += " ";
    out += buf;
  }
  out += "]";
  emit_raw(key, out);
}

inline void emit_iarray(const std::string& key,
                        const std::vector<long long>& v) {
  std::string out = "[";
  char buf[32];
  for (size_t i = 0; i < v.size(); ++i) {
    std::snprintf(buf, sizeof(buf), "%lld", v[i]);
    if (i) out += " ";
    out += buf;
  }
  out += "]";
  emit_raw(key, out);
}

// Long enum sweeps (tens of thousands of voxel-status queries) are emitted as
// one character per query instead of one key per query: the comparator can
// still name the exact index that diverged, but the result file stays small
// enough to read.
inline void emit_codes(const std::string& key, const std::string& codes) {
  emit_s(key, codes);
}

}  // namespace parity

#endif  // GBP_PARITY_EMIT_H_
