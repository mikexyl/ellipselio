#include "map_processing.h"
#include <pcl/point_types.h>

namespace ellipselio {
void MappingNode::ExportResearchFrame() {
  if (research_cloud_->empty()) throw std::runtime_error("empty synchronized full cloud");
  const auto stamp = kf_state_.time;
  if (stamp != research_stamp_) throw std::runtime_error("LiDAR update changed deskew reference timestamp");
  const auto& state = kf_state_.state;
  const std::string body = research_robot_ + "/imu";
  const std::string lidar = research_robot_ + "/lidar";
  const std::string world = research_robot_ + "/odom_ellipselio";
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(research_cloud_->size());
  for (const auto& p : *research_cloud_) {
    pcl::PointXYZI q; q.x = p.x; q.y = p.y; q.z = p.z; q.intensity = p.intensity;
    cloud.push_back(q);
  }
  sensor_msgs::msg::PointCloud2 msg; pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp; msg.header.frame_id = lidar;
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp; odom.header.frame_id = world; odom.child_frame_id = body;
  odom.pose.pose.position.x = state.pos.x(); odom.pose.pose.position.y = state.pos.y(); odom.pose.pose.position.z = state.pos.z();
  odom.pose.pose.orientation.x = state.rot.coeffs()[0]; odom.pose.pose.orientation.y = state.rot.coeffs()[1];
  odom.pose.pose.orientation.z = state.rot.coeffs()[2]; odom.pose.pose.orientation.w = state.rot.coeffs()[3];
  std::vector<double> marginal, matrix, rotation, translation;
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = state.rot.toRotationMatrix(); T.block<3,1>(0,3) = state.pos;
  const int tangent[6] = {3,4,5,0,1,2};
  for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) {
    marginal.push_back(kf_state_.cov(tangent[r],tangent[c]));
    odom.pose.covariance[r*6+c] = kf_state_.cov(r,c);
  }
  for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) matrix.push_back(T(r,c));
  for (int r = 0; r < 3; ++r) {
    translation.push_back(state.offset_T_L_I[r]);
    for (int c = 0; c < 3; ++c) rotation.push_back(state.offset_R_L_I.toRotationMatrix()(r,c));
  }
  ResearchExport::Packet packet;
  packet.metadata = {{"schema_version", 1}, {"frontend", "ellipselio"}, {"robot_id", research_robot_},
    {"frame_id", research_frame_++}, {"stamp_ns", stamp.nanoseconds()},
    {"scan_start_ns", scan_start_time_.nanoseconds()}, {"scan_end_ns", scan_end_time_.nanoseconds()},
    {"world_frame", world}, {"body_frame", body}, {"cloud_frame", lidar}, {"cloud_points", cloud.size()},
    {"T_world_body", matrix}, {"calibration", {{"R_body_lidar",rotation},{"t_body_lidar",translation}}},
    {"filter_marginal_rotation_translation", marginal},
    {"covariance_provenance", "EllipseLIO filter marginal; diagnostic only; not a relative edge covariance"},
    {"lidar_updated", research_lidar_updated_}, {"lidar_update_invoked", map_counter_ > 0}, {"visual_update_invoked", false},
    {"visual_correction_enabled", false}, {"image_available", false},
    {"cloud_source", "all valid range-filtered input points in processed interval, deskewed before adaptive filtering"},
    {"pose_source", "EllipseLIO post-LiDAR-update IMU state at exact deskew reference time"}};
  packet.add("/"+research_robot_+"/research/cloud", "sensor_msgs/msg/PointCloud2", msg);
  packet.add("/"+research_robot_+"/research/odometry", "nav_msgs/msg/Odometry", odom);
  const Eigen::Quaterniond rotation_now(state.rot.toRotationMatrix());
  const bool selected = research_ellipsoid_keyframes_ &&
      (research_last_key_ns_ == 0 ||
       (state.pos - research_last_key_pos_).norm() >= research_key_translation_ ||
       research_last_key_rot_.angularDistance(rotation_now) >= research_key_rotation_ * M_PI / 180.0 ||
       (stamp.nanoseconds() - research_last_key_ns_) * 1e-9 >= research_key_interval_);
  if (selected || (research_ellipsoid_next_ < research_ellipsoid_times_.size() &&
      stamp.nanoseconds() >= research_ellipsoid_times_[research_ellipsoid_next_]))
    AddResearchEllipsoids(packet);
  if (selected) {
    research_last_key_ns_ = stamp.nanoseconds();
    research_last_key_pos_ = state.pos;
    research_last_key_rot_ = rotation_now;
  }
  research_export_->enqueue(std::move(packet));
}

void MappingNode::AddResearchEllipsoids(ResearchExport::Packet& packet) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = kf_state_.time;
  msg.header.frame_id = research_robot_ + (research_ellipsoid_world_ ? "/odom_ellipselio" : "/imu");
  msg.height = 1; msg.is_dense = true; msg.is_bigendian = false;
  const std::vector<std::string> names = {"x", "y", "z", "a", "b", "c",
      "v00", "v01", "v02", "v10", "v11", "v12", "v20", "v21", "v22", "map_id", "primitive"};
  for (size_t i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i]; field.offset = 4*i; field.count = 1;
    field.datatype = i < 15 ? sensor_msgs::msg::PointField::FLOAT32 : sensor_msgs::msg::PointField::UINT32;
    msg.fields.push_back(field);
  }
  msg.point_step = 4*names.size();
  const Eigen::Matrix3f R = kf_state_.state.rot.toRotationMatrix().transpose().cast<float>();
  const Eigen::Vector3f t = kf_state_.state.pos.cast<float>();
  for (size_t i = 0; i < map_->map_cloud_->size(); ++i) {
    if (!map_->filters_[i][1]) continue;
    const auto& p = map_->map_cloud_->points[i];
    const Eigen::Vector3f local_center = R*(p.getVector3fMap()-t);
    const Eigen::Vector3f center = research_ellipsoid_world_ ? Eigen::Vector3f(p.getVector3fMap()) : local_center;
    const Eigen::Vector3f axes = map_->eigenvalues_[i];
    const Eigen::Matrix3f basis = research_ellipsoid_world_ ? map_->eigenvectors_[i] : Eigen::Matrix3f(R*map_->eigenvectors_[i]);
    if (local_center.norm() > research_ellipsoid_range_) continue;
    if (!center.allFinite() || !axes.allFinite() || !basis.allFinite() || axes.minCoeff() <= 0)
      throw std::runtime_error("Invalid fitted map ellipsoid");
    // These are native geometric semi-axes, not covariance eigenvalues.
    float values[15] = {center.x(), center.y(), center.z(), axes.x(), axes.y(), axes.z()};
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) values[6+3*r+c] = basis(r,c);
    const size_t offset = msg.data.size(); msg.data.resize(offset+msg.point_step);
    std::memcpy(msg.data.data()+offset, values, sizeof(values));
    const uint32_t id = i, primitive = p.prim_type;
    std::memcpy(msg.data.data()+offset+60, &id, 4);
    std::memcpy(msg.data.data()+offset+64, &primitive, 4);
    ++msg.width;
  }
  msg.row_step = msg.width*msg.point_step;
  std::vector<int64_t> requested;
  if (research_ellipsoid_keyframes_) requested.push_back(kf_state_.time.nanoseconds());
  while (research_ellipsoid_next_ < research_ellipsoid_times_.size() &&
         research_ellipsoid_times_[research_ellipsoid_next_] <= kf_state_.time.nanoseconds())
    requested.push_back(research_ellipsoid_times_[research_ellipsoid_next_++]);
  packet.metadata["ellipsoids"] = {{"schema_version", 1}, {"requested_stamps_ns", requested},
      {"count", msg.width}, {"max_center_range_m", research_ellipsoid_range_},
      {"frame", msg.header.frame_id}, {"source", "all fitted native map ellipsoids after MapIncremental"},
      {"axes", "a,b,c are geometric semi-axis lengths in metres; columns of v are their orthonormal directions"},
      {"primitive", "85 plane, 170 line, 255 ball"},
      {"causality", "persistent spatial map constructed only from observations processed by stamp_ns"}};
  packet.add("/"+research_robot_+"/research/ellipsoids", "sensor_msgs/msg/PointCloud2", msg);
}
}  // namespace ellipselio
