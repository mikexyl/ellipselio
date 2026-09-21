#include "map_processing.h"
#include <cassert>
#include <iostream>
#include <set>
namespace ellipselio {
struct SubmapTest {
  static std::shared_ptr<MappingNode> spatial_node(const std::string& metric="3d", double max_age=120) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("mapping.submaps.enabled", true),
      rclcpp::Parameter("mapping.submaps.strategy", "spatial"),
      rclcpp::Parameter("mapping.submaps.spatial.radius_m", 4.),
      rclcpp::Parameter("mapping.submaps.spatial.overlap_m", 2.),
      rclcpp::Parameter("mapping.submaps.spatial.distance_metric", metric),
      rclcpp::Parameter("mapping.submaps.spatial.max_age_s", max_age),
      rclcpp::Parameter("mapping.submaps.spatial.min_support_points", 1),
      rclcpp::Parameter("mapping.submaps.spatial.min_support_ratio", .1),
      rclcpp::Parameter("publish.markers", false),
      rclcpp::Parameter("imu.topic", "/test/imu"),
      rclcpp::Parameter("lidar.topic", "/test/lidar"),
      rclcpp::Parameter("lidar.t_imu_lidar", std::vector<double>{0,0,0}),
      rclcpp::Parameter("lidar.r_imu_lidar", std::vector<double>{1,0,0,0,1,0,0,0,1})});
    auto node=std::make_shared<MappingNode>(options);
    node->TimerCallback(); node->kf_state_.cov.setIdentity();
    return node;
  }

  static void prepare_scan(MappingNode& n, int64_t stamp, const V3D& position) {
    n.kf_state_.state.pos=position; n.kf_state_.time=rclcpp::Time(stamp,RCL_ROS_TIME);
    n.scan_end_time_=n.kf_state_.time;
    n.scan_cloud_->clear(); n.scan_cloud_bins_.setZero(); n.start_bin_=0;
    for (int i=0;i<27;++i) {
      EllipseLioPoint p{};p.bin_idx=0;
      p.x=5+(i%3)*.2-position.x();p.y=2+((i/3)%3)*.2-position.y();p.z=3+(i/9)*.2-position.z();
      n.scan_cloud_->push_back(p);
    }
    n.scan_cloud_bins_[0]=27;
  }

  static void insert_scan(MappingNode& n) {
    // Registration/support queries must only see previous scans.
    for (const auto& p:*n.map_->map_cloud_) assert(p.scan_idx<n.map_counter_);
    if (n.successor_started_)
      for (const auto& p:*n.successor_->map_cloud_) assert(p.scan_idx<n.map_counter_);
    n.MapIncremental();++n.map_counter_;
    for (auto* buffer:{n.map_.get(),n.successor_.get()}) {
      if (buffer->members.empty())continue;
      assert(buffer->geometry.size()==27*buffer->members.size());
      for (const auto& p:*buffer->map_cloud_)
        assert(p.scan_idx>=buffer->members.front() && p.scan_idx<=buffer->members.back());
      for (const auto& neighbors:buffer->neighbours_)
        for (int j:neighbors)assert(j>=0 && j<int(buffer->map_cloud_->size()));
      // Supply valid fitted fixtures to isolate scheduler readiness behavior.
      for (size_t j=0;j<buffer->filters_.size();++j) {
        buffer->filters_[j][1]=1;buffer->eigenvalues_[j]=V3F::Constant(.2);
        buffer->eigenvectors_[j]=M3F::Identity();
      }
    }
  }

  static void spatial() {
    constexpr int64_t epoch=100000000000LL;
    std::vector<int> fast,slow;
    for (int64_t dt:{100000000LL,1000000000LL}) {
      auto n=spatial_node();std::set<void*> buffers;
      for (int i=0;i<80;++i) {
        prepare_scan(*n,epoch+i*dt,V3D(i,0,0));
        const auto covariance=n->kf_state_.cov;
        const auto position=n->kf_state_.state.pos;
        n->AdvanceSubmaps(epoch+i*dt);
        assert(n->kf_state_.cov==covariance && n->kf_state_.state.pos==position);
        if (i==2)assert(!n->successor_started_); // threshold observed after insertion
        if (i==3)assert(n->successor_started_ && n->successor_->members.empty());
        if (i==5) {
          assert(n->map_->id==1 && n->submap_event_=="handover_radius");
          assert((n->map_->members==std::vector<int>{3,4}));
          assert(n->successor_->members.front()==0); // retired map never queried
        }
        (dt==100000000LL?fast:slow).push_back(n->map_->id);
        buffers.insert(n->map_.get());buffers.insert(n->successor_.get());
        insert_scan(*n);
      }
      assert(buffers.size()==2 && n->analytics_msg_.handovers>20);
    }
    assert(fast==slow); // distance-based handovers are invariant to replay speed
    {
      auto n=spatial_node();
      for (int i=0;i<6;++i) {
        prepare_scan(*n,epoch+i*100000000LL,V3D(i,0,0));
        if (i==5)for (auto& f:n->successor_->filters_)f[1]=0;
        n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
        if (i==5)assert(n->map_->id==0 && n->submap_event_=="handover_waiting_for_support");
        insert_scan(*n);
      }
      prepare_scan(*n,epoch+600000000LL,V3D(6,0,0));
      n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
      assert(n->map_->id==1 && n->submap_event_=="handover_radius");
      // A large timestamp gap cannot query stale geometry from either buffer.
      prepare_scan(*n,epoch+200000000000LL,V3D(6,0,0));
      const auto covariance=n->kf_state_.cov;
      n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
      assert(n->submap_event_=="max_age_recovery" && n->map_->map_cloud_->empty());
      assert(n->map_->members.empty() && n->map_->tensors_p1_.empty() && n->kf_state_.cov==covariance);
    }
    {
      auto n=spatial_node("horizontal");
      // Freeze the map's gravity plane at its first member, including tilted worlds.
      n->kf_state_.state.grav=S2(V3D(0,9.81,0));
      for (int i=0;i<12;++i) {
        prepare_scan(*n,epoch+i*1000000000LL,V3D(0,i,0));
        n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());insert_scan(*n);
        assert(n->map_->extent_m<1e-10 && n->map_->id==0);
      }
      prepare_scan(*n,epoch+20000000000LL,V3D(1,0,0));
      n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());insert_scan(*n);
      assert(std::abs(n->map_->extent_m-1)<1e-10);
    }
    {
      auto n=spatial_node("3d",20);
      for (int i=0;i<=20;++i) {
        prepare_scan(*n,epoch+i*1000000000LL,V3D::Zero());
        n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
        if(i<20)assert(n->map_->id==0);
        if(i==10)assert(n->successor_started_);
        if(i==20)assert(n->map_->id==1 && n->submap_event_=="handover_max_age");
        insert_scan(*n);
      }
    }
    {
      auto n=spatial_node();
      // Accumulated path length/rotation do not substitute for spatial extent.
      for (int i=0;i<40;++i) {
        prepare_scan(*n,epoch+i*100000000LL,V3D(i%2,0,0));
        n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());insert_scan(*n);
      }
      assert(n->traj_dist_.back()>30 && n->map_->extent_m==1 && n->map_->id==0);
      // At the transport limit, unsupported geometry is retired before append.
      n->map_->geometry.resize(kMaxMapPoints);
      prepare_scan(*n,epoch+5000000000LL,V3D::Zero());
      n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
      assert(n->submap_event_=="payload_capacity_recovery" && n->map_->geometry.empty());
    }
    for(const auto& metric:{"unknown", "xy"}) {
      bool rejected=false;
      try {auto n=spatial_node(metric);}catch(const std::invalid_argument&){rejected=true;}
      assert(rejected);
    }
    std::cout<<"Spatial boundaries, speed independence, gravity plane, readiness, stale recovery, capacity and local ownership passed\n";
  }
  static std::shared_ptr<MappingNode> coverage_node(double age=120, double motion=80, double overlap=.25) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("mapping.submaps.enabled", true),
      rclcpp::Parameter("mapping.submaps.strategy", "coverage"),
      rclcpp::Parameter("mapping.submaps.coverage.cell_size_m", 1.),
      rclcpp::Parameter("mapping.submaps.coverage.start_area_m2", 12.),
      rclcpp::Parameter("mapping.submaps.coverage.target_area_m2", 18.),
      rclcpp::Parameter("mapping.submaps.coverage.start_new_area_m2", 3.),
      rclcpp::Parameter("mapping.submaps.coverage.new_area_m2", 9.),
      rclcpp::Parameter("mapping.submaps.coverage.min_shared_area_m2", 6.),
      rclcpp::Parameter("mapping.submaps.coverage.min_overlap_ratio", overlap),
      rclcpp::Parameter("mapping.submaps.coverage.max_age_s", age),
      rclcpp::Parameter("mapping.submaps.coverage.max_displacement_m", motion),
      rclcpp::Parameter("mapping.submaps.coverage.min_support_points", 1),
      rclcpp::Parameter("mapping.submaps.coverage.min_support_ratio", .1),
      rclcpp::Parameter("publish.markers", false),
      rclcpp::Parameter("imu.topic", "/test/imu"), rclcpp::Parameter("lidar.topic", "/test/lidar"),
      rclcpp::Parameter("lidar.t_imu_lidar", std::vector<double>{0,0,0}),
      rclcpp::Parameter("lidar.r_imu_lidar", std::vector<double>{1,0,0,0,1,0,0,0,1})});
    auto n=std::make_shared<MappingNode>(options); n->TimerCallback(); n->kf_state_.cov.setIdentity();
    n->kf_state_.state.grav=S2(V3D(0,0,-9.81));
    return n;
  }
  static void coverage_scan(MappingNode& n,int64_t stamp,int patch,const V3D& pos=V3D(0,0,30),double yaw=0) {
    n.kf_state_.state.pos=pos;
    n.kf_state_.state.rot=Eigen::Quaterniond(Eigen::AngleAxisd(yaw,V3D::UnitZ()));
    n.kf_state_.time=rclcpp::Time(stamp,RCL_ROS_TIME); n.scan_end_time_=n.kf_state_.time;
    n.scan_cloud_->clear(); n.scan_cloud_bins_.setZero(); n.start_bin_=0;
    for (int i=0;i<27;++i) {
      const V3D world(50.1+patch+i%3,10.1+(i/3)%3,.1*(i/9));
      const V3D local=n.kf_state_.state.rot.conjugate()*(world-pos);
      EllipseLioPoint p{};p.bin_idx=0;p.x=local.x();p.y=local.y();p.z=local.z();n.scan_cloud_->push_back(p);
    }
    n.scan_cloud_bins_[0]=27;
  }
  static void coverage() {
    constexpr int64_t epoch=100000000000LL;
    {
      auto n=coverage_node();
      // Forward returns lie ~50 m away, below a drone at 30 m. Translation and
      // changed attitude alone do not add occupied scene cells.
      for(int i=0;i<12;++i) {
        coverage_scan(*n,epoch+i*100000000LL,0,V3D(i,0,30+i),0);
        n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());insert_scan(*n);
        assert(n->CoverageArea(*n->map_)==9 && n->CoverageNewArea(*n->map_)==0);
        assert(!n->successor_started_ && n->map_->id==0);
      }
      assert(n->coverage_origin_.x()>49 && n->coverage_origin_.z()<1);
    }
    {
      auto n=coverage_node();std::set<void*> buffers;
      for(int i=0;i<60;++i) {
        // Newly observed patches while the drone stays at the SAME position.
        coverage_scan(*n,epoch+i*100000000LL,i,V3D(40,0,30),.03*i);
        const auto cov=n->kf_state_.cov;const auto pos=n->kf_state_.state.pos;
        if(i==4)assert(n->map_->shared_area_m2==12 && std::abs(n->map_->overlap_ratio-2./3)<1e-12);
        n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
        assert(n->kf_state_.cov==cov && n->kf_state_.state.pos==pos);
        if(i==2)assert(n->successor_started_ && n->successor_->coverage_cells.empty());
        if(i==4) {
          assert(n->submap_event_=="handover_coverage" && n->map_->id==1);
          assert(n->CoverageArea(*n->map_)==12 && n->CoverageNewArea(*n->map_)==3);
        }
        buffers.insert(n->map_.get());buffers.insert(n->successor_.get());
        insert_scan(*n);
        for (auto* b:{n->map_.get(),n->successor_.get()})
          for (const auto& c:b->coverage_cells)
            assert(std::find(b->members.begin(),b->members.end(),c.second)!=b->members.end());
      }
      assert(buffers.size()==2 && n->analytics_msg_.handovers>10 && n->map_->extent_m==0);
    }
    {
      auto n=coverage_node(120,80,.9);
      for(int i=0;i<5;++i) {
        coverage_scan(*n,epoch+i*100000000LL,i);
        n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
        if(i==4)assert(n->submap_event_=="coverage_waiting_for_overlap" && n->map_->id==0);
        insert_scan(*n);
      }
    }
    {
      auto n=coverage_node(10);
      for(int i=0;i<=10;++i) {
        coverage_scan(*n,epoch+i*1000000000LL,0);
        n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
        if(i==10)assert(n->submap_event_=="handover_max_age" && n->map_->id==1);
        insert_scan(*n);
      }
      coverage_scan(*n,epoch+200000000000LL,0);
      const auto cov=n->kf_state_.cov;n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
      assert(n->submap_event_=="max_age_recovery" && n->map_->coverage_cells.empty() && n->kf_state_.cov==cov);
    }
    {
      auto n=coverage_node(120,4);
      for(int i=0;i<6;++i) {
        coverage_scan(*n,epoch+i*100000000LL,0,V3D(i,0,30));
        n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
        if(i==5)assert(n->submap_event_=="handover_max_displacement");
        insert_scan(*n);
      }
    }
    {
      auto n=coverage_node();n->kf_state_.state.grav=S2(V3D(0,-9.81,0));
      double first_area=0;
      for(int i=0;i<8;++i) {
        coverage_scan(*n,epoch+i*100000000LL,0);
        for(auto& p:*n->scan_cloud_)p.y+=i*3; // Changing height along gravity is not new XY coverage.
        n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());insert_scan(*n);
        if(i==0)first_area=n->CoverageArea(*n->map_);
        assert(n->CoverageArea(*n->map_)==first_area && n->CoverageNewArea(*n->map_)==0);
      }
    }
    std::cout<<"Observed-area gain, stationary new views, translated repeated views, scene-centered frame, measured overlap, guards and two-buffer ownership passed\n";
  }
  static void run() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("mapping.submaps.enabled", true),
      rclcpp::Parameter("publish.markers", false),
      rclcpp::Parameter("imu.topic", "/test/imu"),
      rclcpp::Parameter("lidar.topic", "/test/lidar"),
      rclcpp::Parameter("lidar.t_imu_lidar", std::vector<double>{0,0,0}),
      rclcpp::Parameter("lidar.r_imu_lidar", std::vector<double>{1,0,0,0,1,0,0,0,1})});
    auto node = std::make_shared<MappingNode>(options);
    node->TimerCallback();
    node->kf_state_.cov.setIdentity();
    constexpr int64_t epoch = 100000000000LL;
    const auto position = node->kf_state_.state.pos;
    const auto covariance = node->kf_state_.cov;
    std::set<void*> buffers;
    for (int scan = 0; scan < 400; ++scan) {
      const int64_t stamp = epoch + scan*100000000LL;
      node->AdvanceSubmaps(stamp);
      buffers.insert(node->map_.get()); buffers.insert(node->successor_.get());
      const int active = scan < 100 ? 0 : (scan-100)/50+1;
      assert(node->map_->id == active);
      assert(node->successor_started_ == (scan >= 50));
      // The new scan cannot be in the matching map yet.
      for (const auto& p : *node->map_->map_cloud_) assert(p.scan_idx < scan);
      node->scan_cloud_->clear();
      node->scan_cloud_bins_.setZero();
      for (int i=0; i<27; ++i) {
        EllipseLioPoint p{};
        p.x=1+(i%3)*.2f; p.y=2+((i/3)%3)*.2f; p.z=3+(i/9)*.2f;
        p.bin_idx=0; node->scan_cloud_->push_back(p);
      }
      node->scan_cloud_bins_[0]=27;
      node->start_bin_=0;
      node->scan_end_time_=rclcpp::Time(stamp,RCL_ROS_TIME);
      node->kf_state_.time=node->scan_end_time_;
      node->MapIncremental();
      node->map_counter_++;
      for (const auto& p : *node->map_->map_cloud_) {
        assert(p.scan_idx >= active*50);
        assert(p.scan_idx <= scan);
      }
      for (const auto& list : node->map_->neighbours_)
        for (int index : list) assert(index>=0 && index<int(node->map_->map_cloud_->size()));
      assert(node->map_->members.front()==active*50);
      assert(node->map_->members.back()==scan);
      assert(node->map_->geometry.size()==node->map_->members.size()*27);
      assert(node->kf_state_.state.pos == position);
      assert(node->kf_state_.cov == covariance);
    }
    assert(buffers.size()==2);
    node->AdvanceSubmaps(epoch+100000000000LL);
    assert(node->map_->begin_ns==epoch+95000000000LL);
    assert(node->map_->map_cloud_->empty());
    assert(node->map_->tensors_p1_.empty());
    assert(node->map_->members.empty());
    // Empty/one-point scans fail without a statistical reduction.
    esekfom::dyn_share_datastruct<double> data;
    node->scan_cloud_->clear(); node->TensorRegistration(node->kf_state_.state,data);
    assert(!data.valid);
    node->scan_cloud_->push_back(EllipseLioPoint{});
    node->TensorRegistration(node->kf_state_.state,data); assert(!data.valid);
    // A genuinely one-feature match (two scan points, only one within the
    // unchanged correspondence radius) must fail before sample variance.
    node->scan_cloud_->clear();
    EllipseLioPoint point{}; point.x=1; point.y=0; point.z=0; point.bin_idx=0;
    node->scan_cloud_->push_back(point);
    node->scan_cloud_bins_.setZero(); node->scan_cloud_bins_[0]=1;
    node->scan_end_time_=rclcpp::Time(epoch+100000000000LL,RCL_ROS_TIME);
    node->MapIncremental();
    node->map_counter_++;
    node->scan_end_time_=rclcpp::Time(epoch+100100000000LL,RCL_ROS_TIME);
    node->map_->filters_[0][1]=1;
    node->map_->salivalues_[0]=V3F(1,0,0);
    node->map_->eigenvectors_[0]=M3F::Identity();
    point.z=.02f; node->scan_cloud_->points[0]=point;
    point.x=100; point.y=100; point.z=100; node->scan_cloud_->push_back(point);
    node->mean_bin_=1; node->ekfom_iter_cnt_=0;
    node->TensorRegistration(node->kf_state_.state,data);
    assert(!data.valid); assert(node->analytics_msg_.num_feats==1);
    // A nonfinite tensor cannot enter correspondence statistical reductions.
    node->map_->salivalues_[0][0]=std::numeric_limits<float>::quiet_NaN();
    node->TensorRegistration(node->kf_state_.state,data);
    assert(!data.valid); assert(node->analytics_msg_.num_feats==0);
    // Disabled scheduling leaves persistent geometry, tensors and ownership intact.
    auto* persistent=node->map_.get();
    const size_t persistent_size=node->map_->map_cloud_->size();
    node->submaps_enabled_=false;
    node->AdvanceSubmaps(epoch+1000000000000LL);
    assert(node->map_.get()==persistent);
    assert(node->map_->map_cloud_->size()==persistent_size);
    std::cout << "Window boundaries, overlap, map-local indices, member geometry, two-buffer reuse, gaps, state preservation and empty scans passed\n";
  }
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv); ellipselio::SubmapTest::run(); ellipselio::SubmapTest::spatial(); ellipselio::SubmapTest::coverage(); rclcpp::shutdown();
}
