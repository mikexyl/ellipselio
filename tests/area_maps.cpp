#include "map_processing.h"
#include <cassert>
#include <iostream>
namespace ellipselio {
struct AreaMapTest {
 static void run() {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("mapping.area_maps.enabled",true),
   rclcpp::Parameter("mapping.area_maps.robot","Bob"),rclcpp::Parameter("mapping.area_maps.radius_m",10.),
   rclcpp::Parameter("imu.topic","/test/imu"),rclcpp::Parameter("lidar.topic","/test/lidar"),
   rclcpp::Parameter("lidar.t_imu_lidar",std::vector<double>{0,0,0}),
   rclcpp::Parameter("lidar.r_imu_lidar",std::vector<double>{1,0,0,0,1,0,0,0,1})});
  auto n=std::make_shared<MappingNode>(options);
  n->scan_times_={1000000000LL,2000000000LL,3000000000LL};
  n->kf_state_.state.pos=V3D(3,4,40);n->kf_state_.state.grav=S2(V3D(0,0,-9.81));
  n->kf_state_.state.rot=Eigen::Quaterniond(Eigen::AngleAxisd(.4,V3D::UnitZ()));
  n->kf_state_.cov.setIdentity();n->kf_state_.time=rclcpp::Time(3000000000LL,RCL_ROS_TIME);
  n->scan_end_time_=rclcpp::Time(4000000000LL,RCL_ROS_TIME); // staged but not inserted
  for (const auto& v:std::vector<V3F>{{3,4,-500},{13,4,700},{13.001f,4,0},{2,3,20}}) {
    EllipseLioPoint p{};p.getVector3fMap()=v;p.scan_idx=n->map_cloud_->size()%3;n->map_cloud_->push_back(p);
    n->filters_.push_back(Eigen::Vector2i::Ones());n->eigenvalues_.push_back(V3F::Constant(.2));n->eigenvectors_.push_back(M3F::Identity());
  }
  const auto before=n->kf_state_.state.pos;const auto cov=n->kf_state_.cov;const auto geometry=*n->map_cloud_;
  const auto ids=n->SelectAreaPoints(before,V3D::UnitZ());assert((ids==std::vector<int>{0,1,3}));
  assert(n->SelectAreaPoints(V3D(100,100,40),V3D::UnitZ()).empty());
  assert(n->SelectAreaPoints(before,V3D::UnitZ())==ids); // no age/height eviction on return
  auto packet=n->AreaSnapshot("test");
  assert(packet.metadata["anchor_sensor_ns"]==3000000000LL);
  assert(packet.metadata["geometry_count"]==3 && packet.metadata["schema_version"]==7 && packet.messages.size()==3);
  assert(packet.metadata["odometry_map_source"]=="persistent_map" && packet.metadata["age_limit_s"].is_null());
  assert(packet.metadata["member_scan_ids"]==std::vector<int>({0,1}));
  assert(packet.metadata["available_ns"]>=packet.metadata["stamp_ns"]);
  const auto* saved=reinterpret_cast<const float*>(packet.messages[0].data());
  const auto* saved_ids=reinterpret_cast<const int32_t*>(packet.messages[1].data());
  for(size_t i=0;i<ids.size();++i) {
    assert(saved_ids[i]==ids[i]);
    const V3D restored=n->kf_state_.state.rot*Eigen::Map<const V3F>(saved+3*i).cast<double>()+before;
    assert((restored-geometry[ids[i]].getVector3fMap().cast<double>()).norm()<1e-4);
  }
  assert(n->kf_state_.state.pos==before && n->kf_state_.cov==cov);
  assert(n->map_cloud_->size()==geometry.size());
  const auto immutable=packet.messages;n->map_cloud_->points[0].x+=1000;assert(packet.messages==immutable);
  n->map_cloud_->points[0].x=std::numeric_limits<float>::quiet_NaN();
  bool rejected=false;try{n->SelectAreaPoints(before,V3D::UnitZ());}catch(const std::runtime_error&){rejected=true;}assert(rejected);
  for(const auto& setting:{"mapping.submaps.enabled","mapping.area_maps.odometry"}) {
    auto invalid=options;invalid.append_parameter_override(setting,true);rejected=false;
    try{auto other=std::make_shared<MappingNode>(invalid);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
  }
  std::cout<<"Export-only area membership, all ages/heights, boundary, return visits, anchor transforms, immutable packets, unchanged state/covariance, nonfinite rejection and odometry-policy checks passed\n";
 }
}; }
int main(int argc,char** argv){rclcpp::init(argc,argv);ellipselio::AreaMapTest::run();rclcpp::shutdown();}
