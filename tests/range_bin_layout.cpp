// Diagnostic against the installed frontend: does scan assembly preserve the
// contiguous range-bin slices assumed by TensorRegistration?
#include "map_processing.h"
#include <iostream>
#include <cassert>
namespace ellipselio {
struct AreaMapTest {
  static void run() {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("imu.topic","/probe/imu"),
      rclcpp::Parameter("lidar.topic","/probe/lidar"),rclcpp::Parameter("lidar.type",2),
      rclcpp::Parameter("lidar.scan_lines",16),
      rclcpp::Parameter("lidar.t_imu_lidar",std::vector<double>{0,0,0}),
      rclcpp::Parameter("lidar.r_imu_lidar",std::vector<double>{1,0,0,0,1,0,0,0,1})});
    auto n=std::make_shared<MappingNode>(options);
    n->TimerCallback();
    n->imu_process_->imu_start_time_=rclcpp::Time(1000000000LL,RCL_ROS_TIME);
    n->imu_process_->imu_end_time_=rclcpp::Time(2000000000LL,RCL_ROS_TIME);
    n->buffer_start_time_=rclcpp::Time(1100000000LL,RCL_ROS_TIME);
    n->buffer_end_time_=rclcpp::Time(1200000000LL,RCL_ROS_TIME);
    n->raw_start_time_=rclcpp::Time(1200000000LL,RCL_ROS_TIME);
    n->raw_end_time_=rclcpp::Time(1300000000LL,RCL_ROS_TIME);
    n->start_bin_=4;n->mean_bin_=12;
    for (int bin:{4,20}) {
      EllipseLioPoint p{};p.x=bin;p.bin_idx=bin;p.time_secs=1;p.time_nsecs=150000000;
      n->buffer_cloud_->push_back(p);n->buffer_cloud_bins_[bin]=1;
      p.time_nsecs=250000000;n->raw_cloud_->push_back(p);n->raw_cloud_bins_[bin]=1;
    }
    const auto original_buffer = *n->buffer_cloud_;
    const auto original_raw = *n->raw_cloud_;
    const auto original_buffer_bins = n->buffer_cloud_bins_;
    const auto original_raw_bins = n->raw_cloud_bins_;
    for (int case_id=0; case_id<3; ++case_id) {
      *n->buffer_cloud_=original_buffer; *n->raw_cloud_=original_raw;
      n->buffer_cloud_bins_=original_buffer_bins; n->raw_cloud_bins_=original_raw_bins;
      n->buffer_start_time_=rclcpp::Time(1100000000LL,RCL_ROS_TIME);
      n->raw_start_time_=rclcpp::Time(1200000000LL,RCL_ROS_TIME);
      n->raw_end_time_=rclcpp::Time(1300000000LL,RCL_ROS_TIME);
      n->imu_process_->imu_start_time_=rclcpp::Time(1000000000LL,RCL_ROS_TIME);
      n->imu_process_->imu_end_time_=rclcpp::Time(case_id==1?1200000000LL:2000000000LL,RCL_ROS_TIME);
      if(case_id==2) n->imu_process_->imu_start_time_=rclcpp::Time(1200000000LL,RCL_ROS_TIME);
      n->SyncRawCloudWithImu();
    std::cout<<"actual point bins:";
    for(const auto& p:*n->scan_cloud_)std::cout<<" "<<p.bin_idx;
    std::cout<<"\nregistration assumes:";
    for(int b=0;b<n->scan_cloud_bins_.size();++b)
      for(int i=0;i<n->scan_cloud_bins_[b];++i)std::cout<<" "<<b;
    int wrong=0,offset=0;
    for(int b=0;b<n->scan_cloud_bins_.size();++b)
      for(int i=0;i<n->scan_cloud_bins_[b];++i)
        wrong+=n->scan_cloud_->points[offset++].bin_idx!=b;
    std::cout<<"\nmismatched bin assignments: "<<wrong<<" / "<<offset<<"\n";
    assert(wrong==0);
    assert(offset==(case_id==0?4:2));
    assert(n->scan_cloud_bins_.sum()==n->scan_cloud_->size());
    for(int i=0;i<offset;++i) {
      const auto& point=n->scan_cloud_->points[i];
      assert(point.x==point.bin_idx);
      if(case_id==0) assert(point.time_nsecs==(i%2?250000000:150000000));
      else assert(point.time_nsecs==(case_id==1?150000000:250000000));
    }
    assert(n->buffer_cloud_->size()==(case_id==1?2:0));
    }
    n->raw_cloud_->clear();n->buffer_cloud_->clear();
    n->SyncRawCloudWithImu();assert(n->scan_cloud_->empty() && n->scan_cloud_bins_.sum()==0);
  }
};
}
int main(int argc,char** argv){rclcpp::init(argc,argv);ellipselio::AreaMapTest::run();rclcpp::shutdown();}
