#pragma once
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
namespace ellipselio {
// Gravity/bias initialization decided from sensor timestamps only, never from
// callback or timer interleaving, so every replay of an input initializes
// identically. `stamps` are the buffered IMU stamps in arrival (= stamp) order.
struct InitializationDecision {
  enum Kind { Wait, ScanBeforeInitialization, Initialize } kind;
  // Initialize: average the first `averaged` samples; the next sample, still
  // before the scan start, becomes the first propagated state.
  size_t averaged = 0;
};
inline InitializationDecision DecideInitialization(const std::vector<int64_t>& stamps,
                                                   int64_t scan_start, int rate) {
  if (rate < 1) throw std::invalid_argument("IMU rate must be positive");
  // Every sample before the scan start is known only once one at or after it arrived.
  if (stamps.empty() || stamps.back() < scan_start) return {InitializationDecision::Wait};
  size_t before = 0;
  while (before < stamps.size() && stamps[before] < scan_start) ++before;
  // The original contract averages more than `rate` samples (> 1 s); one more
  // sample before the scan start propagates the state into the scan.
  if (before < static_cast<size_t>(rate) + 2)
    return {InitializationDecision::ScanBeforeInitialization};
  return {InitializationDecision::Initialize, before - 1};
}
}  // namespace ellipselio
