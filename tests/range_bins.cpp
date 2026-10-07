#include "range_bins.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <numeric>
#include <random>

struct Point {
  unsigned bin_idx;
  int acquisition_id;
};

void verify(std::vector<Point> points, std::size_t bins) {
  const auto original = points;
  std::vector<int> counts(bins, -1);
  std::vector<Point> scratch;
  ellipselio::GroupRangeBins(points, counts, scratch);
  assert(std::accumulate(counts.begin(), counts.end(), 0) == int(points.size()));
  std::size_t offset = 0;
  for (std::size_t bin = 0; bin < bins; ++bin) {
    std::vector<int> expected, actual;
    for (const auto& point : original)
      if (point.bin_idx == bin) expected.push_back(point.acquisition_id);
    for (int n = 0; n < counts[bin]; ++n) {
      assert(points[offset].bin_idx == bin);
      actual.push_back(points[offset++].acquisition_id);
    }
    assert(actual == expected);  // No lost, duplicated, or reordered observations.
  }
  assert(!ellipselio::GroupRangeBins(points, counts, scratch));
}

int main() {
  // Two internally ordered scans are not ordered when concatenated.
  verify({{1, 0}, {1, 1}, {4, 2}, {1, 3}, {4, 4}, {7, 5}}, 10);
  verify({}, 10);
  verify({{0, 1}}, 10);
  verify({{9, 1}, {9, 2}}, 10);
  std::mt19937 random(51);
  std::vector<Point> points;
  for (int i = 0; i < 50000; ++i)
    points.push_back({static_cast<unsigned>(random() % 100), i});
  verify(points, 100);
  std::vector<int> counts(3, 77);
  points = {{0, 1}, {3, 2}};
  std::vector<Point> scratch;
  bool rejected = false;
  try { ellipselio::GroupRangeBins(points, counts, scratch); }
  catch (const std::out_of_range&) { rejected = true; }
  assert(rejected && counts[0] == 77 && points[0].acquisition_id == 1);
  std::cout << "Range-bin boundaries, stable membership, empty, repeated and invalid input passed\n";
}
