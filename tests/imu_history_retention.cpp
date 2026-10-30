#include "imu_processing.h"
#include "scan_readiness.h"
#include <atomic>
#include <cassert>
#include <iostream>
#include <thread>

struct ImuHistoryRetentionTest {
  static constexpr int64_t epoch = 100000000000LL;
  static constexpr int64_t step = 2000000LL;  // Original 500 Hz samples.
  static rclcpp::Time at(int64_t offset) { return rclcpp::Time(epoch+offset,RCL_ROS_TIME); }
  static ImuParams params() {
    ImuParams p{}; p.rate=500; p.gyr_noise=.01; p.acc_noise=.1;
    p.gyr_bias=p.acc_bias=.0001; p.topic="/test/imu";
    p.t_imu_lidar=V3D::Zero(); p.r_imu_lidar=M3D::Identity(); return p;
  }
  static IkfomSPtr filter() {
    IkfomSPtr kf(new Ikfom());
    kf->init_dyn_share(GetF,DfDx,DfDw,
      [](state_ikfom&,esekfom::dyn_share_datastruct<double>& d) {
        // Same deterministic observation in the old eager and new lazy paths.
        // Covariance still updates, so future covariance repropagation matters.
        d.valid=true; d.h=Eigen::VectorXd::Zero(6);
        d.h_x=Eigen::MatrixXd::Identity(6,6); d.h_x_R=Eigen::MatrixXd::Identity(6,6);
      },10,1e-3);
    return kf;
  }
  static void feed(ImuProcess& imu,int first,int last,bool eager=false) {
    for(int i=first;i<=last;++i) {
      auto msg=std::make_unique<sensor_msgs::msg::Imu>(); msg->header.stamp=at(i*step);
      msg->linear_acceleration.z=kGravityMetersPerSecondSquared;
      msg->linear_acceleration.x=.1*std::sin(i*.01);
      msg->angular_velocity.z=.02*std::sin(i*.003);
      imu.ImuCallback(std::move(msg));
      // This recreates the original callback's immediate prediction of every
      // sample, without maintaining a second estimator implementation.
      if(eager) imu.FinishInput();
    }
  }
  static EllipseLioPointCloudPtr cloud(int64_t end) {
    EllipseLioPointCloudPtr p(new EllipseLioPointCloud);
    for(int k=0;k<10;++k) {
      EllipseLioPoint q{};q.x=1.f+k*.1f;q.y=.2f;q.bin_idx=1;
      const auto t=epoch+end-49000000LL+k*5000000LL;
      q.time_secs=t/1000000000LL;q.time_nsecs=t%1000000000LL;p->push_back(q);
    }
    return p;
  }
  static void same(const KfState& a,const KfState& b) {
    assert(a.time==b.time);
    assert((a.state.pos-b.state.pos).norm()<1e-9);
    assert((a.state.vel-b.state.vel).norm()<1e-9);
    assert((a.state.rot.toRotationMatrix()-b.state.rot.toRotationMatrix()).norm()<1e-9);
    assert((a.state.bg-b.state.bg).norm()<1e-9 && (a.state.ba-b.state.ba).norm()<1e-9);
    assert((a.state.grav.get_vect()-b.state.grav.get_vect()).norm()<1e-9);
    assert((a.cov-b.cov).cwiseAbs().maxCoeff()<1e-9);
  }
  static void run() {
    auto node=std::make_shared<rclcpp::Node>("imu_retention_test");
    ImuProcess lazy(filter(),params(),node),eager(filter(),params(),node);
    lazy.ReleaseHistoryBefore(at(0));
    feed(lazy,0,6000);feed(eager,0,6000,true);  // 12 s delivered before mapping.
    // The first scan (start 1.201 s) initializes from the 600 samples before
    // 1.200 s; sample 600 (1.200 s) is the first propagated state.
    const auto first_start=at(1201000000LL);
    assert(lazy.InitializeBefore(first_start)==ellipselio::InitializationDecision::Initialize);
    assert(eager.InitializeBefore(first_start)==ellipselio::InitializationDecision::Initialize);
    eager.FinishInput();
    assert(!lazy.imu_need_init_ && lazy.imu_states_.empty());
    assert(lazy.Diagnostics().pending_samples>5000 && lazy.Diagnostics().prediction_steps==0);
    rclcpp::Time oldest,newest;
    uint64_t previous_predictions=0;
    int64_t last_end=0;
    for(int scan=0;scan<25;++scan) {
      const int64_t end=1251000000LL+scan*50000000LL,start=end-50000000LL;
      last_end=end;
      lazy.SyncWithLidar(&oldest,&newest,at(end));
      assert(ellipselio::CompleteScanReadiness(epoch+start,epoch+end,
        oldest.nanoseconds(),newest.nanoseconds(),true)==ellipselio::ScanReadiness::Ready);
      assert(newest>at(end) && (newest-at(end)).nanoseconds()<=step);
      eager.SyncWithLidar(&oldest,&newest,at(end));
      auto a=cloud(end),b=cloud(end);KfState ka,kb;
      lazy.UndistortPointCloud(a,&ka,at(start),at(end),{});
      eager.UndistortPointCloud(b,&kb,at(start),at(end),{});
      same(ka,kb);
      for(size_t k=0;k<a->size();++k)
        assert((a->points[k].getVector3fMap()-b->points[k].getVector3fMap()).norm()<1e-6f);
      lazy.UpdateStatesWithLidar(&ka,at(end),1.);
      eager.UpdateStatesWithLidar(&kb,at(end),1.);
      assert(lazy.trial_lidar_updated && eager.trial_lidar_updated);same(ka,kb);
      auto work=lazy.Diagnostics();
      assert(work.repropagation_steps==0 && work.copied_states<=526);
      if(scan) assert(work.prediction_steps-previous_predictions<=26);
      previous_predictions=work.prediction_steps;
      lazy.ReleaseHistoryBefore(at(start));eager.ReleaseHistoryBefore(at(start));
    }
    assert(eager.Diagnostics().repropagation_steps>100000);
    assert(lazy.Diagnostics().peak_materialized_states<=526);

    // Future arrivals remain raw samples: a much larger backlog must not
    // increase scan-local prediction/repropagation or materialized-state size.
    const auto before=lazy.Diagnostics();
    std::thread producer([&]{feed(lazy,6001,50000);});
    for(int i=0;i<10;++i) lazy.SyncWithLidar(&oldest,&newest,at(last_end));
    producer.join();
    lazy.SyncWithLidar(&oldest,&newest,at(last_end));
    auto after=lazy.Diagnostics();
    assert(after.received_samples==50001 && after.pending_samples>48000);
    assert(after.prediction_steps==before.prediction_steps && after.repropagation_steps==0);
    assert(after.materialized_states<=526 && after.copied_states<=526);
    lazy.FinishInput();
    auto finished=lazy.Diagnostics();
    assert(finished.pending_samples==0);
    assert(finished.prediction_steps+600==finished.received_samples);
    assert(lazy.imu_states_.back().state.time==at(50000*step));
    lazy.ReleaseHistoryBefore(at(50000*step));assert(lazy.imu_states_.size()==500);

    // A scan waiting for future coverage survives late IMU delivery, and
    // materialization stops at its exact original bracket once data arrives.
    ImuProcess delayed(filter(),params(),node);
    feed(delayed,0,1500);
    assert(delayed.InitializeBefore(first_start)==ellipselio::InitializationDecision::Initialize);
    const auto start=at(3990000000LL),end=at(4041000000LL);
    delayed.SyncWithLidar(&oldest,&newest,end);
    const auto protected_oldest=oldest;
    assert(ellipselio::CompleteScanReadiness(start.nanoseconds(),end.nanoseconds(),
      oldest.nanoseconds(),newest.nanoseconds(),true)==ellipselio::ScanReadiness::Wait);
    feed(delayed,1501,4000);
    delayed.SyncWithLidar(&oldest,&newest,end);
    assert(oldest==protected_oldest && newest>end && (newest-end).nanoseconds()<=step);
    assert(delayed.Diagnostics().pending_samples>1900);
    auto p=cloud(4041000000LL);KfState state;
    delayed.UndistortPointCloud(p,&state,start,end,{});
    delayed.UpdateStatesWithLidar(&state,end,1.);assert(delayed.trial_lidar_updated);
    delayed.ReleaseHistoryBefore(start);
    assert(delayed.imu_states_.front().state.time<start);
    assert(delayed.Diagnostics().repropagation_steps==0);
    std::cout<<"Original 500 Hz IMU / 20 Hz LiDAR: eager-versus-lazy pose, covariance and deskew equivalence; "
                "bounded state work under a 100-second arrival backlog; complete samples; concurrent arrivals; "
                "delayed coverage and predecessor retention passed\n";
  }
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);ImuHistoryRetentionTest::run();rclcpp::shutdown();
}
