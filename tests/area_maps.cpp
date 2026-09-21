#include "map_processing.h"
#include <cassert>
#include <iostream>

namespace ellipselio {
struct AreaMapTest {
  static void odometry() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("mapping.area_maps.enabled",true),
      rclcpp::Parameter("mapping.area_maps.odometry",true),
      rclcpp::Parameter("mapping.area_maps.radius_m",10.),
      rclcpp::Parameter("publish.markers",false),
      rclcpp::Parameter("imu.topic","/test/imu"),
      rclcpp::Parameter("lidar.topic","/test/lidar"),
      rclcpp::Parameter("lidar.t_imu_lidar",std::vector<double>{0,0,0}),
      rclcpp::Parameter("lidar.r_imu_lidar",std::vector<double>{1,0,0,0,1,0,0,0,1})});
    auto n=std::make_shared<MappingNode>(options);n->TimerCallback();
    assert(n->area_odometry_ && !n->submaps_enabled_ && !n->successor_ && !n->area_history_);
    n->kf_state_.cov.setIdentity();n->kf_state_.state.grav=S2(V3D(0,0,-9.81));
    auto* map=n->map_.get();
    for (int scan=0;scan<80;++scan) {
      n->kf_state_.state.pos=V3D(scan,0,140);n->kf_state_.time=rclcpp::Time(100000000000LL+scan*1000000000LL,RCL_ROS_TIME);
      n->scan_end_time_=n->kf_state_.time;n->scan_cloud_->clear();n->scan_cloud_bins_.setZero();n->start_bin_=0;
      for (int i=0;i<27;++i) {
        EllipseLioPoint p{};p.bin_idx=0;p.x=5+(i%3)*.2-scan;p.y=2+((i/3)%3)*.2;p.z=3+(i/9)*.2-140;
        n->scan_cloud_->push_back(p);
      }
      n->scan_cloud_bins_[0]=27;
      const auto covariance=n->kf_state_.cov;const auto position=n->kf_state_.state.pos;
      n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
      assert(n->kf_state_.cov==covariance && n->kf_state_.state.pos==position && n->map_.get()==map);
      for (const auto& p:*map->map_cloud_)assert(p.scan_idx<scan);
      n->MapIncremental();++n->map_counter_;
    }
    assert(n->analytics_msg_.handovers==0 && map->members.empty() && map->geometry.empty());
    assert(map->map_cloud_->points.front().scan_idx==0);
    std::vector<int> ids;std::vector<float> distances;
    const V3F query(5.05,2,3);
    n->FindOdometryNeighbors(query,1,ids,distances);assert(ids.empty()); // Drone is far away.
    n->kf_state_.state.pos=V3D(5,2,140);n->AdvanceSubmaps(200000000000LL);
    n->FindOdometryNeighbors(query,1,ids,distances);
    assert(!ids.empty() && map->map_cloud_->points[ids[0]].scan_idx==0); // Old point re-enters at any height.
    n->kf_state_.state.pos=V3D(100,0,0); // An iEKF pose change cannot move the query area mid-update.
    n->FindOdometryNeighbors(query,1,ids,distances);assert(!ids.empty());
    n->AdvanceSubmaps(201000000000LL);n->FindOdometryNeighbors(query,1,ids,distances);assert(ids.empty());

    // Closest outside point must not mask the next closest inside point.
    iOctree::Octree tree;tree.SetBucketSize(1);tree.SetMaxOctants(10000);tree.SetMaxNewPoints(1000);tree.SetMinExtent(.01);
    EllipseLioPointCloud cloud;
    for(float x:{10.1f,10.f,9.8f}) {EllipseLioPoint p{};p.x=x;p.z=500;cloud.push_back(p);}
    std::vector<int> added,map_ids;tree.Update(cloud,added,map_ids,0,cloud.size(),.01);
    auto in_area=[&](int id){return cloud[added[id]].x<=10;};
    tree.KnnNeighborsIf(V3F(10.09,0,500),1,ids,distances,1,in_area);
    assert(ids.size()==1 && cloud[added[ids[0]]].x==10.f);
    tree.KnnNeighborsIf(V3F(10.09,0,500),1,ids,distances,.05,in_area);assert(ids.empty());

    // Random, tilted cylinder membership: compare filtered octree search to brute force.
    tree.clear();cloud.clear();added.clear();map_ids.clear();
    std::mt19937 random(41);std::uniform_real_distribution<float> uniform(-12,12);
    for(int i=0;i<200;++i){EllipseLioPoint p{};p.x=uniform(random);p.y=uniform(random);p.z=uniform(random);cloud.push_back(p);}
    tree.Update(cloud,added,map_ids,0,cloud.size(),.01);
    const V3F up=V3F(1,2,3).normalized();
    auto allowed=[&](int id){const V3F p=cloud[added[id]].getVector3fMap();return (p-up*up.dot(p)).squaredNorm()<=100;};
    for(int i=0;i<128;++i) {
      const V3F q(uniform(random),uniform(random),uniform(random));float best=25;int expected=-1;
      for(size_t j=0;j<added.size();++j) {
        const float d=(cloud[added[j]].getVector3fMap()-q).squaredNorm();
        if(allowed(j) && d>0 && d<=25 && (expected<0 || d<best)){best=d;expected=j;}
      }
      tree.KnnNeighborsIf(q,1,ids,distances,5,allowed);
      assert(expected<0 ? ids.empty() : ids.size()==1 && ids[0]==expected);
    }
    for(const std::string setting:{"mapping.submaps.enabled","mapping.area_maps.enabled"}) {
      auto invalid=options;invalid.append_parameter_override(setting,setting=="mapping.submaps.enabled");
      bool rejected=false;try{auto other=std::make_shared<MappingNode>(invalid);}catch(const std::invalid_argument&){rejected=true;}
      assert(rejected);
    }
    std::cout<<"Area odometry: persistent ownership, no handovers/self-match, old return geometry, unbounded height, frozen query region, boundary search and randomized nearest-neighbor oracle passed\n";
  }
  static void run() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("mapping.submaps.enabled",true),
      rclcpp::Parameter("mapping.area_maps.enabled",true),
      rclcpp::Parameter("mapping.area_maps.radius_m",10.),
      rclcpp::Parameter("mapping.area_maps.robot","A"),
      rclcpp::Parameter("publish.markers",false),
      rclcpp::Parameter("imu.topic","/test/imu"),
      rclcpp::Parameter("lidar.topic","/test/lidar"),
      rclcpp::Parameter("lidar.t_imu_lidar",std::vector<double>{0,0,0}),
      rclcpp::Parameter("lidar.r_imu_lidar",std::vector<double>{1,0,0,0,1,0,0,0,1})});
    auto n=std::make_shared<MappingNode>(options);n->TimerCallback();
    assert(n->area_history_ && n->area_history_.get()!=n->map_.get());
    auto* history=n->area_history_.get();
    n->kf_state_.cov.setIdentity();
    for (int scan=0;scan<80;++scan) {
      n->kf_state_.state.pos=V3D(scan,0,40);n->kf_state_.time=rclcpp::Time(100000000000LL+scan*1000000000LL,RCL_ROS_TIME);
      n->scan_end_time_=n->kf_state_.time;n->scan_cloud_->clear();n->scan_cloud_bins_.setZero();n->start_bin_=0;
      for (int i=0;i<27;++i) {
        EllipseLioPoint p{};p.bin_idx=0;p.x=5+(i%3)*.2-scan;p.y=2+((i/3)%3)*.2;p.z=3+(i/9)*.2-40;
        n->scan_cloud_->push_back(p);
      }
      n->scan_cloud_bins_[0]=27;n->AdvanceSubmaps(n->scan_end_time_.nanoseconds());
      const auto covariance=n->kf_state_.cov;const auto position=n->kf_state_.state.pos;
      for (const auto& p:*n->map_->map_cloud_)assert(p.scan_idx<scan);
      n->MapIncremental();++n->map_counter_;
      assert(n->area_history_.get()==history && n->map_.get()!=history);
      assert(n->kf_state_.cov==covariance && n->kf_state_.state.pos==position);
      assert(history->members.empty() && history->geometry.empty());
      for (const auto& neighbors:history->neighbours_)
        for (int j:neighbors)assert(j>=0 && size_t(j)<history->map_cloud_->size());
    }
    assert(n->analytics_msg_.handovers>10);
    assert(history->map_cloud_->points.front().scan_idx==0);
    for (auto& p:*n->map_->map_cloud_)assert(p.scan_idx>=70); // Odometry is still recent.
    const size_t count=history->map_cloud_->size();assert(count>0);
    const V3D up=V3D::UnitZ(),center(5,2,140);
    auto near=n->SelectAreaPoints(*history,center,up);assert(near.size()==count);
    assert(n->SelectAreaPoints(*history,V3D(100,0,140),up).empty());
    assert(n->SelectAreaPoints(*history,center,up)==near); // Leaving/returning never ages out points.
    assert(n->SelectAreaPoints(*history,V3D(5,100,3),V3D::UnitY()).size()==count);
    // Exact inclusion of boundary, exclusion just outside, independent of age/height.
    MappingNode::MapBuffer boundary;
    for (float x:{10.f,10.001f}) {EllipseLioPoint p{};p.x=x;p.z=-500;boundary.map_cloud_->push_back(p);}
    assert((n->SelectAreaPoints(boundary,V3D::Zero(),up)==std::vector<int>{0}));
    n->kf_state_.state.pos=center;n->kf_state_.state.grav=S2(V3D(0,0,-9.81));
    for (size_t i=0;i<count;++i) {
      history->filters_[i][1]=1;history->eigenvalues_[i]=V3F::Constant(.2);history->eigenvectors_[i]=M3F::Identity();
    }
    n->scan_end_time_=rclcpp::Time(n->scan_times_.back()+100000000LL,RCL_ROS_TIME);
    auto packet=n->AreaSnapshot("test");
    assert(packet.metadata["anchor_sensor_ns"]==n->scan_times_.back());
    assert(packet.metadata["geometry_count"]==count && packet.metadata["ellipsoid_count"]==count);
    assert(packet.metadata["last_member_ns"]<packet.metadata["anchor_sensor_ns"]);
    assert(packet.metadata["age_limit_s"].is_null() && packet.messages.size()==5);
    assert(packet.messages[2].size()==count*sizeof(int32_t));
    const auto saved=packet.messages;
    history->map_cloud_->points.front().x+=1000;
    assert(packet.messages==saved); // Writer packet owns immutable geometry and IDs.
    std::cout<<"Accumulated map survives odometry handovers; all in-area points retained regardless of age/height; return visits, frame transforms, identities, immutable packets and filter state passed\n";
  }
};
}
int main(int argc,char** argv) {rclcpp::init(argc,argv);ellipselio::AreaMapTest::run();ellipselio::AreaMapTest::odometry();rclcpp::shutdown();}
