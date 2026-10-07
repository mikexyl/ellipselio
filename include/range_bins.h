#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace ellipselio {
// Partial scans are concatenated by arrival, but covariance and octree updates
// address each range bin by a contiguous prefix-count interval. Restore that
// invariant without changing the points or their order within a range bin.
template <class Points, class Counts>
bool GroupRangeBins(Points& points, Counts& counts, Points& scratch) {
  const std::size_t bins = counts.size();
  std::vector<std::size_t> sizes(bins, 0);
  bool ordered = true;
  std::size_t previous = 0;
  for (const auto& point : points) {
    const std::size_t bin = point.bin_idx;
    if (bin >= bins) throw std::out_of_range("Invalid point range bin");
    ordered = ordered && bin >= previous;
    previous = bin;
    ++sizes[bin];
  }
  for (std::size_t bin = 0; bin < bins; ++bin) counts[bin] = sizes[bin];
  if (ordered) return false;
  std::vector<std::size_t> offsets(bins, 0);
  for (std::size_t bin = 1; bin < bins; ++bin)
    offsets[bin] = offsets[bin - 1] + sizes[bin - 1];
  scratch.resize(points.size());
  for (const auto& point : points) scratch[offsets[point.bin_idx]++] = point;
  points.swap(scratch);
  return true;
}
}  // namespace ellipselio
