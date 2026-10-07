#include "map_processing.h"
#ifdef ELLIPSELIO_AREA_EXPORT
namespace ellipselio {
void MappingNode::ConfigureAreaExports() {
  fresh_observations_=declare_parameter<bool>("mapping.area_maps.fresh_observations",false);
  accumulated_with_observations_=declare_parameter<bool>("mapping.area_maps.accumulated_with_observations",false);
  if (accumulated_with_observations_ && !fresh_observations_)
    throw std::invalid_argument("Combined exports require fresh observation tracking");
  area_maps_enabled_=declare_parameter<bool>("mapping.area_maps.enabled",false);
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
      const auto python=declare_parameter<std::string>("mapping.area_maps.python","");
      const auto writer=declare_parameter<std::string>("mapping.area_maps.writer","");
      area_export_=std::make_unique<SnapshotWriter>(python,writer,output,2);
      if (accumulated_with_observations_)
        accumulated_export_=std::make_unique<SnapshotWriter>(python,writer,output+"_accumulated",2);
    }
  }
}
void MappingNode::CloseMappingOutputs() {
  if (outputs_closed_) return;
  if (accumulated_with_observations_) MaybeExportArea(true);
  if (fresh_observations_) ExportFreshObservations(true); else MaybeExportArea(true);
  if (accumulated_export_) accumulated_export_->close();
  if (area_export_) area_export_->close();
  if (diagnostics_.is_open()) { diagnostics_.flush(); if (!diagnostics_) throw std::runtime_error("Native log flush failed"); diagnostics_.close(); }
  outputs_closed_=true;
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

// The corrected current scan is exported once, after MapIncremental transformed
// it. No estimator/map membership or IMU-thread-owned state is modified.
void MappingNode::ExportFreshObservations(bool shutdown) {
  if (!area_export_) return;
  if (!shutdown) {
    const auto& state=kf_state_.state;
    const M3D rotation=state.rot.toRotationMatrix();
    std::vector<double> transform(16,0.); transform[15]=1.;
    for(int r=0;r<3;++r) {
      for(int c=0;c<3;++c) transform[4*r+c]=rotation(r,c);
      transform[4*r+3]=state.pos[r];
    }
    const int64_t stamp=scan_end_time_.nanoseconds();
    if (observation_metadata_.empty()) observation_begin_ns_=stamp;
    observation_metadata_.push_back({{"scan_id",map_counter_},
      {"stamp_ns",kf_state_.time.nanoseconds()},{"sensor_stamp_ns",stamp},
      {"scan_start_ns",scan_start_time_.nanoseconds()},
      {"acquisition_floor_ns",imu_start_time_.nanoseconds()}, {"T_world_imu",transform}});
    for (const auto& point:scan_cloud_->points) {
      const int64_t acquisition=int64_t(point.time_secs)*1000000000LL+point.time_nsecs;
      if (!point.getVector3fMap().allFinite() || acquisition<scan_start_time_.nanoseconds() || acquisition>stamp)
        throw std::runtime_error("Invalid fresh processed observation");
      observation_xyz_.insert(observation_xyz_.end(),{point.x,point.y,point.z});
      observation_ids_.push_back(observation_next_id_++);
      observation_scans_.push_back(map_counter_); observation_times_.push_back(acquisition);
    }
    if (stamp-observation_begin_ns_<1000000000LL) return;
  }
  if (observation_metadata_.empty()) return;
  const auto& scan=observation_metadata_.back();
  const V3D gravity=kf_state_.state.grav.get_vect();
  SnapshotWriter::Packet packet;
  packet.metadata={{"schema_version",2},{"strategy","processed_scans"},
    {"frame","native_odom"},{"robot_id",area_robot_},{"submap_id",observation_chunk_id_++},
    {"complete",true},{"odometry_map_source","persistent_map"},
    {"geometry_count",observation_ids_.size()},
    {"gravity_world_m_s2",{gravity.x(),gravity.y(),gravity.z()}},
    {"available_ns",std::max(scan["stamp_ns"].get<int64_t>(),scan["sensor_stamp_ns"].get<int64_t>())},
    {"watermark_ns",scan["acquisition_floor_ns"]},{"watermark_semantics","imu_history_lower_bound"},
    {"scans",observation_metadata_}};
  if (accumulated_with_observations_) packet.metadata["accumulated_snapshots"]=area_next_id_;
  auto add=[&packet](const auto& values) {
    using T=typename std::decay_t<decltype(values)>::value_type;
    packet.messages.emplace_back(values.size()*sizeof(T));
    if (!values.empty()) std::memcpy(packet.messages.back().data(),values.data(),values.size()*sizeof(T));
  };
  add(observation_xyz_);add(observation_ids_);add(observation_scans_);add(observation_times_);
  area_export_->enqueue(std::move(packet));
  observation_xyz_.clear(); observation_ids_.clear();observation_scans_.clear();observation_times_.clear();
  observation_metadata_=nlohmann::json::array();
}
} // namespace ellipselio
#endif
