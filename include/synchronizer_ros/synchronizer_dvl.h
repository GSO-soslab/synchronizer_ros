#ifndef SYNCHRONIZER_DVL_H_
#define SYNCHRONIZER_DVL_H_

#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <map>
#include <vector>

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>

#include <synchronizer_ros/TimeNumbered.h>
#include "ds_sensor_msgs/NortekDF3.h"
#include "ds_sensor_msgs/NortekDF21.h"
#include "ds_sensor_msgs/Dvl.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Dense>

namespace synchronizer_ros {

#define PI 3.1415926

class SynchronizerDvl {
public:
  SynchronizerDvl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  ~SynchronizerDvl();

  void timeCallback(const synchronizer_ros::TimeNumbered &msg_time);

  void bottomtrackCallback(const ds_sensor_msgs::NortekDF21 &msg_bt);

  void currentprofileCallback(const ds_sensor_msgs::NortekDF3 &msg_cp);

  bool readParameters();

  void associateTimeStampsAndCleanUp();

  void publish(const synchronizer_ros::TimeNumbered &msg_time, 
               ds_sensor_msgs::NortekDF21 &msg_bt);

  void publish(const synchronizer_ros::TimeNumbered &msg_time, 
               ds_sensor_msgs::NortekDF3 &msg_cp);

  void derivedCP(geometry_msgs::PointStamped* depth_data, 
                 ds_sensor_msgs::NortekDF3* big_msg);

  void derivedBT(ds_sensor_msgs::Dvl* dvl_data,
                 geometry_msgs::TwistStamped* velocity_data,
                 geometry_msgs::PointStamped* depth_data, 
                 sensor_msgs::PointCloud2& pointcloud_data,
                 ds_sensor_msgs::NortekDF21* big_msg);
private:
  // ROS members.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber sub_time_;
  ros::Subscriber sub_bt_;
  ros::Subscriber sub_cp_;

  ros::Publisher pub_init_;
  ros::Publisher pub_bt_;
  ros::Publisher pub_cp_;

  // publish derived msg
  ros::Publisher pub_dvl_;
  ros::Publisher pub_velocity_;
  ros::Publisher pub_depth_;
  ros::Publisher pub_pointcloud_;
  
  // Initializing
  bool received_bt_;
  bool received_cp_;
  uint32_t latest_seq_bt_;
  uint32_t latest_seq_cp_;
  bool initialized_;

  ros::Time init_timestamp_;
  synchronizer_ros::TimeNumbered init_time_;

  int64_t offset_;
  int good_matches_;

  // Initialized
  std::vector<synchronizer_ros::TimeNumbered> candidates_time_;
  std::map<uint64_t, ds_sensor_msgs::NortekDF21> candidates_bt_;
  std::map<uint64_t, ds_sensor_msgs::NortekDF3> candidates_cp_;

  // Parameters
  std::string device_name_;
  double sound_speed_;
  double trigger_delay_;
  double delay_threshold_;
  int match_threshold_;
  int max_buffer_;
  int beam_angle_;
  bool pub_derived_msg_;
  
  std::mutex mutex_;

  // double last_sys_time_;

};

} // namespace synchronizer_ros

#endif // SYNCHRONIZER_DVL_H_
