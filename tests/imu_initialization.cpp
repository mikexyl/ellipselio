#include "imu_initialization.h"

#include <cassert>
#include <iostream>

using namespace ellipselio;

int main() {
  const int rate = 10;
  std::vector<int64_t> stamps;
  for (int i = 0; i < 30; ++i) stamps.push_back(1000 + 100 * i);  // 10 Hz, 1000..3900

  // Not every pre-scan sample has arrived yet: wait, whatever arrived so far.
  for (size_t arrived = 0; arrived <= 16; ++arrived) {
    std::vector<int64_t> partial(stamps.begin(), stamps.begin() + arrived);
    assert(DecideInitialization(partial, 2550, rate).kind == InitializationDecision::Wait);
  }
  // A scan with fewer than rate + 2 earlier samples is before initialization,
  // regardless of how many later samples have arrived.
  for (size_t arrived = 11; arrived <= 30; ++arrived) {
    std::vector<int64_t> partial(stamps.begin(), stamps.begin() + arrived);
    if (partial.back() < 2050) continue;
    assert(DecideInitialization(partial, 2050, rate).kind ==
           InitializationDecision::ScanBeforeInitialization);
  }
  // Enough earlier samples: the same split however many later samples arrived.
  for (size_t arrived = 17; arrived <= 30; ++arrived) {
    std::vector<int64_t> partial(stamps.begin(), stamps.begin() + arrived);
    const auto decision = DecideInitialization(partial, 2550, rate);
    assert(decision.kind == InitializationDecision::Initialize);
    assert(decision.averaged == 15);  // 1000..2400 averaged, 2500 propagates into the scan
  }
  // A sample exactly at the scan start does not belong to the initialization.
  assert(DecideInitialization(stamps, 2200, rate).averaged == 11);
  std::cout << "deterministic IMU initialization ok\n";
}
