#include "map_processing.h"
#ifdef ELLIPSELIO_RESEARCH_EXPORT
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
      area_export_=std::make_unique<ResearchExport>(declare_parameter<std::string>("mapping.area_maps.python",""),
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
    {"active_submap_id",-1},{"successor_submap_id",-1},{"handovers",0},
    {"active_points",map_cloud_->size()},{"successor_points",0},
    {"age_mean_s",nullptr},{"age_max_s",nullptr},{"correspondence_age_measured",false},
    {"processing_s",core_time+export_time},{"core_processing_s",core_time},{"snapshot_s",export_time},
    {"odometry_map_source","persistent_map"},{"submap_strategy","disabled"},{"submap_event",""},
    {"pose",{s.pos.x(),s.pos.y(),s.pos.z(),s.rot.coeffs()[0],s.rot.coeffs()[1],s.rot.coeffs()[2],s.rot.coeffs()[3]}}};
  diagnostics_<<row.dump()<<'\n'; diagnostics_.flush();
  if (!diagnostics_) throw std::runtime_error("Native update log write failed");
}
} // namespace ellipselio
#endif
