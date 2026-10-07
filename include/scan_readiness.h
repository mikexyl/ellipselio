#pragma once
#include <cstdint>
#include <stdexcept>
namespace ellipselio {
enum class ScanReadiness { Wait, BeforeInitialization, Ready, Expired };
inline ScanReadiness CompleteScanReadiness(int64_t start, int64_t end,
                                         int64_t imu_start, int64_t imu_end,
                                         bool initialized) {
  if (start > end) throw std::invalid_argument("Reversed LiDAR scan interval");
  if (imu_start >= imu_end) return ScanReadiness::Wait;
  if (start < imu_start)
    return initialized ? ScanReadiness::Expired : ScanReadiness::BeforeInitialization;
  if (end > imu_end) return ScanReadiness::Wait;
  return ScanReadiness::Ready;
}
}  // namespace ellipselio
