#include "lidar_processing.h"
#include "scan_readiness.h"
#include <cassert>
#include <iostream>
struct LidarQueueTest {
  static void run() {
    auto node = std::make_shared<rclcpp::Node>("scan_queue_test");
    LidarParams params{2, 10, 16, 2., 100., 30., "/test/lidar"};
    LidarProcess lidar(params, .1, node);
    auto enqueue = [&](int id) {
      lidar.ellipselio_pc_->clear();
      EllipseLioPoint point{};
      point.x = id; point.bin_idx = 7; point.time_secs = id;
      lidar.ellipselio_pc_->push_back(point);
      lidar.bin_pcs_sizes_.setZero(); lidar.bin_pcs_sizes_[7] = 1;
      lidar.lidar_start_time_ = rclcpp::Time(id*1000000000LL, RCL_ROS_TIME);
      lidar.lidar_end_time_ = rclcpp::Time(id*1000000000LL+100000000, RCL_ROS_TIME);
      lidar.start_bin_ = 2; lidar.mean_bin_ = 7;
      lidar.lidar_time_offset_ = rclcpp::Duration(0, id);
      lidar.QueueProcessedCloud();
    };
    enqueue(1); enqueue(2); enqueue(3);
    // A later callback may mutate all producer buffers; queued scans stay fixed.
    lidar.ellipselio_pc_->clear(); lidar.ClearBins();
    EllipseLioPointCloudPtr points(new EllipseLioPointCloud);
    Eigen::ArrayXi bins; rclcpp::Time start, end; int first, mean; double offset;
    for (int id = 1; id <= 3; ++id) {
      assert(lidar.GetPointCloud(points, &start, &end, &bins, &first, &mean, &offset));
      assert(points->size() == 1 && points->front().x == id);
      assert(points->front().time_secs == unsigned(id) && bins[7] == 1);
      assert(start.nanoseconds() == id*1000000000LL && end.nanoseconds() == id*1000000000LL+100000000);
      assert(first == 2 && mean == 7 && std::abs(offset-id*1e-9) < 1e-12);
      if (id < 3) {
        bool rejected = false;
        try { lidar.GetPointCloud(points, &start, &end, &bins, &first, &mean, &offset); }
        catch (const std::logic_error&) { rejected = true; }
        assert(rejected && lidar.pending_scans_.size() == unsigned(3-id));
      }
      points->clear();
    }
    assert(!lidar.GetPointCloud(points, &start, &end, &bins, &first, &mean, &offset));
    for (int id = 1; id <= 32; ++id) enqueue(id);
    bool rejected = false;
    try { enqueue(33); } catch (const std::runtime_error&) { rejected = true; }
    assert(rejected && lidar.pending_scans_.size() == 32);
  }
};
int main(int argc, char** argv) {
  using namespace ellipselio;
  assert(CompleteScanReadiness(10, 20, 0, 0, false) == ScanReadiness::Wait);
  for (int watermark = 11; watermark < 20; ++watermark)
    assert(CompleteScanReadiness(10, 20, 10, watermark, true) == ScanReadiness::Wait);
  assert(CompleteScanReadiness(10, 20, 10, 20, true) == ScanReadiness::Ready);
  assert(CompleteScanReadiness(10, 20, 11, 30, false) == ScanReadiness::BeforeInitialization);
  assert(CompleteScanReadiness(10, 20, 11, 30, true) == ScanReadiness::Expired);
  rclcpp::init(argc, argv); LidarQueueTest::run(); rclcpp::shutdown();
  std::cout << "FIFO immutable scans, no append, bounded queue, delayed IMU and exact boundaries passed\n";
}
