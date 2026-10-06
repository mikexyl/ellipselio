#include "map_processing.h"
#ifdef ELLIPSELIO_AREA_EXPORT
namespace ellipselio {
void MappingNode::ConfigureAreaExports() {
  area_maps_enabled_=declare_parameter<bool>("mapping.area_maps.enabled",false);
  const auto source=declare_parameter<std::string>("mapping.area_maps.source","persistent_area");
  if (source!="persistent_area" && source!="processed_scans")
    throw std::invalid_argument("Unknown observation export source");
  fresh_observations_=source=="processed_scans";
  area_radius_m_=declare_parameter<double>("mapping.area_maps.radius_m",80.);
  area_step_m_=declare_parameter<double>("mapping.area_maps.snapshot_step_m",20.);
  const double interval=declare_parameter<double>("mapping.area_maps.snapshot_interval_s",10.);
  if (!std::isfinite(area_radius_m_) || area_radius_m_<=0 || !std::isfinite(area_step_m_) || area_step_m_<=0 ||
      !std::isfinite(interval) || interval<=0) throw std::invalid_argument("Invalid area-map export region or cadence");
  area_interval_ns_=llround(interval*1e9);
  const auto diagnostic_path=declare_parameter<std::string>("mapping.diagnostics_path","");
  if (!diagnostic_path.empty()) {
    diagnostics_.open(diagnostic_path);
    if (!diagnostics_) throw std::runtime_error("Cannot open native update log");
  }
  if (area_maps_enabled_) {
    area_robot_=declare_parameter<std::string>("mapping.area_maps.robot","");
    const auto output=declare_parameter<std::string>("mapping.area_maps.output","");
    if (!output.empty()) {
      if (area_robot_.empty()) throw std::invalid_argument("Area-map robot ID required");
      area_export_=std::make_unique<SnapshotWriter>(declare_parameter<std::string>("mapping.area_maps.python",""),
          declare_parameter<std::string>("mapping.area_maps.writer",""),output,2);
    }
  }
}
void MappingNode::CloseMappingOutputs() {
  if (outputs_closed_) return;
  if (fresh_observations_) FlushProcessedScans(); else MaybeExportArea(true);
  if (area_export_) area_export_->close();
  if (diagnostics_.is_open()) { diagnostics_.flush(); if (!diagnostics_) throw std::runtime_error("Native log flush failed"); diagnostics_.close(); }
  outputs_closed_=true;
}

void MappingNode::ExportProcessedScan() {
  if (!fresh_observations_ || !area_export_) return;
  // MapIncremental has transformed this scan exactly once, but has not removed
  // the returns rejected by persistent-map novelty filtering from scan_cloud_.
  const auto sensor_ns=scan_end_time_.nanoseconds();
  if (observation_chunk_id_+observation_poses_.size()>0 && sensor_ns<=observation_last_sensor_ns_)
    throw std::runtime_error("Nonchronological processed observation stream");
  const auto& state=kf_state_.state;
  const M3D rotation=state.rot.toRotationMatrix();
  std::vector<double> pose(16,0.); pose[15]=1.;
  for (int r=0;r<3;++r) {
    for (int c=0;c<3;++c) pose[4*r+c]=rotation(r,c);
    pose[4*r+3]=state.pos[r];
  }
  if (observation_poses_.empty()) observation_first_ns_=sensor_ns;
  // A scan end is not an acquisition watermark: adjacent LiDAR packets can
  // overlap in time. SyncRawCloudWithImu clips every processed return to the
  // monotonic IMU history lower bound. Future scans cannot retain older data.
  const int64_t floor_ns=imu_start_time_.nanoseconds();
  if (floor_ns<observation_watermark_ns_) throw std::runtime_error("Regressing observation watermark");
  observation_poses_.push_back({{"scan_id",map_counter_},{"sensor_stamp_ns",sensor_ns},
    {"scan_start_ns",scan_start_time_.nanoseconds()},{"acquisition_floor_ns",floor_ns},
    {"stamp_ns",kf_state_.time.nanoseconds()},{"T_world_imu",pose}});
  for (const auto& point:scan_cloud_->points) {
    const auto p=point.getVector3fMap();
    const int64_t ns=int64_t(point.time_secs)*1000000000LL+point.time_nsecs;
    if (!p.allFinite() || ns>sensor_ns || ns<scan_start_time_.nanoseconds())
      throw std::runtime_error("Invalid processed observation provenance");
    observation_xyz_.insert(observation_xyz_.end(),p.data(),p.data()+3);
    observation_ids_.push_back(observation_next_id_++);
    observation_scans_.push_back(map_counter_); observation_times_.push_back(ns);
  }
  observation_watermark_ns_=floor_ns; observation_last_sensor_ns_=sensor_ns;
  // Time and point limits bound a packet independently of bag duration.
  if (sensor_ns-observation_first_ns_>=1000000000LL || observation_ids_.size()>=1000000)
    FlushProcessedScans();
}

void MappingNode::FlushProcessedScans() {
  if (!area_export_ || observation_poses_.empty()) return;
  const V3D gravity=kf_state_.state.grav.get_vect();
  SnapshotWriter::Packet packet;
  packet.metadata={{"schema_version",2},{"strategy","processed_scans"},
    {"robot_id",area_robot_},{"submap_id",observation_chunk_id_++},{"complete",true},
    {"geometry_count",observation_ids_.size()},{"watermark_ns",observation_watermark_ns_},
    {"watermark_semantics","imu_history_lower_bound"},
    {"available_ns",std::max(observation_last_sensor_ns_,kf_state_.time.nanoseconds())},
    {"frame","native_odom"},{"odometry_map_source","persistent_map"},
    {"scans",observation_poses_},{"gravity_world_m_s2",{gravity.x(),gravity.y(),gravity.z()}}};
  auto add=[&packet](const auto& values) {
    using T=typename std::decay_t<decltype(values)>::value_type;
    packet.messages.emplace_back(values.size()*sizeof(T));
    if (!values.empty()) std::memcpy(packet.messages.back().data(),values.data(),values.size()*sizeof(T));
  };
  add(observation_xyz_);add(observation_ids_);add(observation_scans_);add(observation_times_);
  area_export_->enqueue(std::move(packet));
  observation_xyz_.clear();observation_ids_.clear();observation_scans_.clear();observation_times_.clear();
  observation_poses_=nlohmann::json::array();
}
void MappingNode::RecordNativeUpdate(double core_time,double export_time) {
  const auto& s=kf_state_.state;
  const bool updated=map_counter_ && imu_process_->trial_lidar_updated && analytics_msg_.num_feats>0 && std::isfinite(analytics_msg_.res_mean);
  analytics_msg_.stamp_ns=kf_state_.time.nanoseconds();
  analytics_msg_.sensor_stamp_ns=scan_end_time_.nanoseconds();
  analytics_msg_.lidar_updated=updated;
  analytics_msg_.odometry_map_source="persistent_map";
  analytics_msg_.snapshot_time=export_time;
  if (!diagnostics_.is_open()) return;
  nlohmann::json row={{"scan_id",map_counter_},{"stamp_ns",kf_state_.time.nanoseconds()},
    {"sensor_stamp_ns",scan_end_time_.nanoseconds()},{"lidar_updated",updated},
    {"upstream_reported_update",map_counter_ && imu_process_->trial_lidar_updated},
    {"features",analytics_msg_.num_feats},{"num_feats",analytics_msg_.num_feats},{"residual",analytics_msg_.res_mean},
    {"processing_s",core_time+export_time},{"core_processing_s",core_time},{"snapshot_s",export_time},
    {"pose",{s.pos.x(),s.pos.y(),s.pos.z(),s.rot.coeffs()[0],s.rot.coeffs()[1],s.rot.coeffs()[2],s.rot.coeffs()[3]}}};
  diagnostics_<<row.dump()<<'\n'; diagnostics_.flush();
  if (!diagnostics_) throw std::runtime_error("Native update log write failed");
}
} // namespace ellipselio
#endif
