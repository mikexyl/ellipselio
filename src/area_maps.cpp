#include "map_processing.h"
#include <set>

namespace ellipselio {

void MappingNode::PrepareAreaOdometry(int64_t stamp) {
  // Freeze the region at the propagated pose for the entire iterated update.
  area_query_center_=kf_state_.state.pos;
  area_query_up_=-kf_state_.state.grav.get_vect().normalized();
  if (!area_query_center_.allFinite() || !area_query_up_.allFinite())
    throw std::runtime_error("Invalid accumulated-area odometry frame");
  if (map_->id<0) {map_->id=0;map_->begin_ns=stamp;}
  submap_event_.clear();
}

void MappingNode::FindOdometryNeighbors(const V3F& query, float radius,
    std::vector<int>& ids, std::vector<float>& distances) {
  if (!area_odometry_) {
    map_->ioctree_.KnnNeighbors(query,1,ids,distances,radius);
    return;
  }
  // Filter candidates inside the octree search, so an outside nearest point
  // cannot hide a slightly farther valid point inside the area. IDs/tensors
  // remain in the persistent map; there is no copied or re-fitted query map.
  map_->ioctree_.KnnNeighborsIf(query,1,ids,distances,radius,[this](int id) {
    const V3D d=map_->map_cloud_->points[id].getVector3fMap().cast<double>()-area_query_center_;
    return (d-area_query_up_*area_query_up_.dot(d)).squaredNorm()<=area_radius_m_*area_radius_m_;
  });
}

std::vector<int> MappingNode::SelectAreaPoints(const MapBuffer& source,
    const V3D& center, const V3D& up) const {
  if (!center.allFinite() || !up.allFinite() || std::abs(up.norm()-1.)>1e-8)
    throw std::runtime_error("Invalid area-map frame");
  std::vector<int> ids;
  // Query the entire retained map. Scan ID and height never gate membership.
  for (size_t i=0; i<source.map_cloud_->size(); ++i) {
    const V3D d=source.map_cloud_->points[i].getVector3fMap().cast<double>()-center;
    if (!d.allFinite()) throw std::runtime_error("Non-finite accumulated map point");
    const V3D horizontal=d-up*up.dot(d);
    if (horizontal.squaredNorm()<=area_radius_m_*area_radius_m_) ids.push_back(i);
  }
  return ids;
}

#ifdef ELLIPSELIO_RESEARCH_EXPORT
ResearchExport::Packet MappingNode::AreaSnapshot(const std::string& reason) {
  const auto& source=area_history_ ? *area_history_ : *map_;
  const auto& state=kf_state_.state;
  const V3D up=-state.grav.get_vect().normalized();
  const auto ids=SelectAreaPoints(source,state.pos,up);
  const M3D rotation=state.rot.toRotationMatrix();
  const M3F R=rotation.transpose().cast<float>();
  const V3F t=state.pos.cast<float>();
  std::vector<float> geometry,ellipsoids;
  std::vector<int32_t> point_ids,scan_ids,ellipsoid_ids;
  std::set<int> scans;
  std::vector<std::vector<int>> ranges;
  geometry.reserve(3*ids.size());
  for (int i:ids) {
    const auto& point=source.map_cloud_->points[i];
    const V3F q=R*(point.getVector3fMap()-t);
    if (!q.allFinite() || point.scan_idx<0 || size_t(point.scan_idx)>=scan_times_.size())
      throw std::runtime_error("Invalid area-map point provenance");
    geometry.insert(geometry.end(),q.data(),q.data()+3);
    point_ids.push_back(i);scan_ids.push_back(point.scan_idx);scans.insert(point.scan_idx);
    if (ranges.empty() || ranges.back()[1]!=i) ranges.push_back({i,i+1});
    else ranges.back()[1]=i+1;
    if (!source.filters_[i][1]) continue;
    const auto& axes=source.eigenvalues_[i];
    const M3F basis=R*source.eigenvectors_[i];
    if (!axes.allFinite() || !basis.allFinite() || axes.minCoeff()<=0)
      throw std::runtime_error("Invalid accumulated ellipsoid");
    ellipsoid_ids.push_back(i);
    ellipsoids.insert(ellipsoids.end(),q.data(),q.data()+3);
    ellipsoids.insert(ellipsoids.end(),axes.data(),axes.data()+3);
    for (int r=0;r<3;++r) for (int c=0;c<3;++c) ellipsoids.push_back(basis(r,c));
  }
  std::vector<double> pose(16,0);
  for (int r=0;r<3;++r) {
    for (int c=0;c<3;++c) pose[4*r+c]=rotation(r,c);
    pose[4*r+3]=state.pos[r];
  }
  pose[15]=1;
  // SyncPackages may already have staged the next unprocessed scan at shutdown.
  const auto sensor_ns=scan_times_.back(), pose_ns=kf_state_.time.nanoseconds();
  const V3D gravity_world=state.grav.get_vect(),gravity_imu=rotation.transpose()*gravity_world;
  ResearchExport::Packet packet;
  packet.metadata={{"schema_version",5},{"strategy","area"},{"robot_id",area_robot_},
    {"submap_id",area_next_id_},{"keyframe_id",area_next_id_},{"complete",true},
    {"retrievable",!ellipsoid_ids.empty()},{"finish_reason",reason},
    {"member_scan_ids",std::vector<int>(scans.begin(),scans.end())},
    {"membership_semantics","origins of selected persistent map representatives; not consecutive scans"},
    {"geometry_id_ranges",ranges},{"archive_point_count",source.map_cloud_->size()},
    {"anchor_scan_id",scan_times_.size()-1},{"anchor_sensor_ns",sensor_ns},
    {"begin_ns",scans.empty()?sensor_ns:scan_times_.at(*scans.begin())},
    {"last_member_ns",scans.empty()?sensor_ns:scan_times_.at(*scans.rbegin())},
    {"end_ns",sensor_ns+1},{"stamp_ns",pose_ns},{"available_ns",std::max(sensor_ns,pose_ns)},
    {"T_world_imu",pose},{"frame","snapshot_anchor_imu"},
    {"gravity_world_m_s2",{gravity_world.x(),gravity_world.y(),gravity_world.z()}},
    {"gravity_imu_m_s2",{gravity_imu.x(),gravity_imu.y(),gravity_imu.z()}},
    {"gravity_source","ellipselio_filter_at_anchor"},
    {"area_center_world",{state.pos.x(),state.pos.y(),state.pos.z()}},
    {"area_up_world",{up.x(),up.y(),up.z()}},{"area_radius_m",area_radius_m_},
    {"area_shape","horizontal_disk_unbounded_height"},{"age_limit_s",nullptr},
    {"geometry_source","all stored native accumulated-map representatives inside area; not full-resolution raw geometry"},
    {"odometry_map_source",area_odometry_?"accumulated_area":submaps_enabled_?"recent_submap":"persistent_map"},
    {"ellipsoid_support","native persistent-map tensor neighborhoods; selected by center, surface clipped to area"},
    {"geometry_count",point_ids.size()},{"ellipsoid_count",ellipsoid_ids.size()}};
  auto add=[&packet](const auto& values) {
    using T=typename std::decay_t<decltype(values)>::value_type;
    packet.messages.emplace_back(values.size()*sizeof(T));
    if (!values.empty()) std::memcpy(packet.messages.back().data(),values.data(),values.size()*sizeof(T));
  };
  add(geometry);add(ellipsoids);add(point_ids);add(scan_ids);add(ellipsoid_ids);
  return packet;
}

void MappingNode::MaybeExportArea(bool shutdown) {
  if (!area_export_ || scan_times_.empty()) return;
  const int64_t now=scan_times_.back();
  if (now==area_last_ns_) return; // Finish must not duplicate the last snapshot.
  const V3D up=-kf_state_.state.grav.get_vect().normalized();
  const V3D d=kf_state_.state.pos-area_last_center_;
  const bool moved=area_last_ns_ && (d-up*up.dot(d)).norm()>=area_step_m_;
  const bool interval=now-(area_last_ns_?area_last_ns_:area_first_ns_)>=area_interval_ns_;
  if (!shutdown && !moved && !interval) return;
  auto packet=AreaSnapshot(shutdown?"shutdown":moved?"displacement":"snapshot_interval");
  area_export_->enqueue(std::move(packet));
  area_last_ns_=now;area_last_center_=kf_state_.state.pos;++area_next_id_;
}
#endif
} // namespace ellipselio
