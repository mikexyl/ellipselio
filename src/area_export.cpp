#include "map_processing.h"
#ifdef ELLIPSELIO_AREA_EXPORT
namespace ellipselio {
void MappingNode::ConfigureAreaExports() {
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
      area_export_=std::make_unique<SnapshotWriter>(declare_parameter<std::string>("mapping.area_maps.python",""),
          declare_parameter<std::string>("mapping.area_maps.writer",""),output,2);
    }
  }
}
void MappingNode::CloseMappingOutputs() {
  if (outputs_closed_) return;
  MaybeExportArea(true);
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
  int offset=0, mismatches=0;
  for(int bin=0;bin<scan_cloud_bins_.size();++bin)
    for(int i=0;i<scan_cloud_bins_[bin];++i,++offset)
      if(offset>=static_cast<int>(scan_cloud_->size()) || scan_cloud_->points[offset].bin_idx!=std::max(bin,start_bin_)) ++mismatches;
  row["effective_range_slice_mismatches"]=mismatches;
  row["range_slice_total"]=offset;
  row["scan_points"]=scan_cloud_->size();
  row["scan_start_ns"]=scan_start_time_.nanoseconds();
  row["imu_buffer_start_ns"]=imu_start_time_.nanoseconds();
  row["imu_buffer_end_ns"]=imu_end_time_.nanoseconds();
  row["buffer_points"]=buffer_cloud_->size();
  row["velocity"]={s.vel.x(),s.vel.y(),s.vel.z()};
  row["gyro_bias"]={s.bg.x(),s.bg.y(),s.bg.z()};
  row["gravity"]={s.grav.get_vect().x(),s.grav.get_vect().y(),s.grav.get_vect().z()};
  diagnostics_<<row.dump()<<'\n'; diagnostics_.flush();
  if (!diagnostics_) throw std::runtime_error("Native update log write failed");
}
} // namespace ellipselio
#endif
