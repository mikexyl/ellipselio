#include "imu_processing.h"
#include "lidar_processing.h"
#include "scan_readiness.h"
#include <atomic>
#include <cassert>
#include <iostream>
#include <thread>

struct ImuHistoryRetentionTest {
  static constexpr int64_t epoch = 100000000000LL;
  static constexpr int64_t step = 2000000LL;  // 500 Hz.
  static rclcpp::Time at(int64_t offset) {
    return rclcpp::Time(epoch + offset, RCL_ROS_TIME);
  }
  static void feed(ImuProcess& imu, int first, int last) {
    for (int i = first; i <= last; ++i) {
      auto msg = std::make_unique<sensor_msgs::msg::Imu>();
      msg->header.stamp = at(i * step);
      msg->linear_acceleration.z = kGravityMetersPerSecondSquared;
      imu.ImuCallback(std::move(msg));
    }
  }
  static void queue(LidarProcess& lidar, int64_t start, int64_t end) {
    lidar.ellipselio_pc_->clear();
    EllipseLioPoint point{};
    point.x = 1.f;
    point.bin_idx = 1;
    point.time_secs = (epoch + end) / 1000000000LL;
    point.time_nsecs = (epoch + end) % 1000000000LL;
    lidar.ellipselio_pc_->push_back(point);
    lidar.bin_pcs_sizes_.setZero();
    lidar.bin_pcs_sizes_[1] = 1;
    lidar.lidar_start_time_ = at(start);
    lidar.lidar_end_time_ = at(end);
    lidar.start_bin_ = lidar.mean_bin_ = 1;
    lidar.QueueProcessedCloud();
  }
  static void run() {
    auto node = std::make_shared<rclcpp::Node>("imu_retention_test");
    IkfomSPtr kf(new Ikfom());
    kf->init_dyn_share(GetF, DfDx, DfDw,
        [](state_ikfom&, esekfom::dyn_share_datastruct<double>& data) {
          // Deterministic zero-residual pose observation, only for exercising
          // the real update/repropagation path after a delayed scan.
          data.valid = true;
          data.h = Eigen::VectorXd::Zero(6);
          data.h_x = Eigen::MatrixXd::Identity(6, 6);
          data.h_x_R = Eigen::MatrixXd::Identity(6, 6);
        }, 10, 1e-3);
    ImuParams params{};
    params.rate = 500;
    params.gyr_noise = .01;
    params.acc_noise = .1;
    params.gyr_bias = params.acc_bias = .0001;
    params.topic = "/test/imu";
    params.t_imu_lidar = V3D::Zero();
    params.r_imu_lidar = M3D::Identity();
    ImuProcess imu(kf, params, node);
    imu.ReleaseHistoryBefore(at(0));  // Empty startup is harmless.
    imu.lidar_ready_ = true;
    feed(imu, 0, 1500);  // Initialize, then accumulate two seconds of states.
    assert(!imu.imu_need_init_ && imu.imu_states_.size() > 500);

    LidarParams lidar_params{3, 20, 64, 0., 100., 45., "/test/lidar"};
    LidarProcess lidar(lidar_params, .1f, node);
    queue(lidar, 1200000000LL, 1250000000LL);
    queue(lidar, 1250000000LL, 1300000000LL);
    queue(lidar, 1300000000LL, 1350000000LL);
    assert(lidar.OldestPendingStart(at(2900000000LL)) == at(1200000000LL));

    EllipseLioPointCloudPtr cloud(new EllipseLioPointCloud);
    rclcpp::Time start, end, oldest, newest;
    Eigen::ArrayXi bins;
    int first, mean;
    double offset;
    assert(lidar.GetPointCloud(cloud, &start, &end, &bins, &first, &mean, &offset));
    imu.SyncWithLidar(&oldest, &newest);
    const auto protected_oldest = oldest;
    assert(ellipselio::CompleteScanReadiness(start.nanoseconds(), end.nanoseconds(),
        oldest.nanoseconds(), newest.nanoseconds(), true) ==
        ellipselio::ScanReadiness::Ready);
    KfState scan_state;
    imu.UndistortPointCloud(cloud, &scan_state, start, end, {});
    assert(scan_state.time >= end && (scan_state.time - end).nanoseconds() <= step);
    assert(std::abs(cloud->front().x - 1.f) < 1e-5);

    // Mapping has dequeued a scan but has not finished its update. Continued
    // IMU callbacks must preserve both its live states and the queued scans.
    std::atomic<bool> done{false};
    std::thread producer([&] { feed(imu, 1501, 2500); done = true; });
    for (int i = 0; i < 20 && !done; ++i) {
      imu.SyncWithLidar(&oldest, &newest);
      assert(oldest == protected_oldest && newest >= at(3000000000LL));
      std::this_thread::yield();
    }
    producer.join();
    assert(imu.imu_states_.front().state.time == protected_oldest);
    imu.UpdateStatesWithLidar(&scan_state, end, 1.);
    assert(imu.trial_lidar_updated);
    assert(scan_state.time >= end && (scan_state.time - end).nanoseconds() <= step);
    assert(imu.imu_states_.back().state.time == at(5000000000LL));
    assert(imu.imu_states_.back().state.cov.allFinite());

    // Release happens after the whole scan cycle, and preserves interpolation
    // history for remaining packets even if the caller supplies a later bound.
    assert(lidar.OldestPendingStart(at(4900000000LL)) == at(1250000000LL));
    imu.ReleaseHistoryBefore(lidar.OldestPendingStart(start));
    assert(imu.imu_states_.front().state.time < start);
    assert(imu.imu_states_[1].state.time >= start);
    assert(imu.imu_states_.size() > 500);
    cloud->clear();
    for (int i = 0; i < 2; ++i) {
      assert(lidar.GetPointCloud(cloud, &start, &end, &bins, &first, &mean, &offset));
      imu.SyncWithLidar(&oldest, &newest);
      assert(oldest < start && newest >= end);
      imu.ReleaseHistoryBefore(lidar.OldestPendingStart(start));
      cloud->clear();
    }
    // Once mapping catches up, excess history is reclaimed. Normal retention
    // is never shorter than the previous nominal sample-count buffer.
    assert(lidar.OldestPendingStart(at(4900000000LL)) == at(4900000000LL));
    imu.ReleaseHistoryBefore(at(4900000000LL));
    assert(imu.imu_states_.size() == 500);

    // A scan waiting for future IMU data remains usable even if delivery later
    // advances several seconds before mapping resumes.
    queue(lidar, 4990000000LL, 5040000000LL);
    assert(lidar.GetPointCloud(cloud, &start, &end, &bins, &first, &mean, &offset));
    imu.SyncWithLidar(&oldest, &newest);
    assert(ellipselio::CompleteScanReadiness(start.nanoseconds(), end.nanoseconds(),
        oldest.nanoseconds(), newest.nanoseconds(), true) ==
        ellipselio::ScanReadiness::Wait);
    feed(imu, 2501, 4000);
    imu.SyncWithLidar(&oldest, &newest);
    assert(oldest < start && newest >= end && imu.imu_states_.size() == 2000);
    imu.UndistortPointCloud(cloud, &scan_state, start, end, {});
    imu.UpdateStatesWithLidar(&scan_state, end, 1.);
    assert(imu.trial_lidar_updated);
    assert(scan_state.time >= end && (scan_state.time - end).nanoseconds() <= step);
    imu.ReleaseHistoryBefore(lidar.OldestPendingStart(start));
    assert(imu.imu_states_.front().state.time < start);
    assert(imu.imu_states_[1].state.time >= start);
    imu.ReleaseHistoryBefore(at(7900000000LL));
    assert(imu.imu_states_.size() == 500);
    std::cout << "500 Hz IMU / 20 Hz LiDAR: queued and in-flight retention, "
                 "concurrent arrivals, delayed deskew/update/repropagation, "
                 "predecessor preservation and catch-up reclamation passed\n";
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ImuHistoryRetentionTest::run();
  rclcpp::shutdown();
}
