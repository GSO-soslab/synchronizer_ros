#ifndef SYNCHRONIZER_DVL_H_
#define SYNCHRONIZER_DVL_H_

#include <cmath>
#include <mutex>
#include <sstream>
#include <string>

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>

#include <synchronizer_ros/ImageNumbered.h>
#include <synchronizer_ros/ImuMicro.h>
#include <synchronizer_ros/TimeNumbered.h>

namespace synchronizer_ros {
  
class Synchronizer {
public:
  Synchronizer(const ros::NodeHandle &nh,
                       const ros::NodeHandle &nh_private);
  ~Synchronizer();

  void imageCallback(const synchronizer_ros::ImageNumbered &image_msg);
  void imageTimeCallback(const synchronizer_ros::TimeNumbered &image_time_msg);
  void publishImg(const synchronizer_ros::ImageNumbered &image_msg);

  bool readParameters();

  // Find nearest time stamp in a list of candidates (only newer are possible
  // due to driver logic).
  void associateTimeStampsAndCleanUp();

private:
  // ROS members.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber image_sub_;
  ros::Subscriber image_time_sub_;

  ros::Publisher initialized_pub_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher image_fast_pub_;

  // Topic names.
  std::string synchronizer_topic_;
  std::string camera_topic_;
  std::string device_name_;
  std::string image_pub_topic_;
  std::string image_time_sub_topic_;
  std::string initialized_pub_topic_;

  // Association members.
  synchronizer_ros::TimeNumbered init_time_;
  std::vector<synchronizer_ros::TimeNumbered> image_time_stamp_candidates_;
  std::vector<synchronizer_ros::ImageNumbered> image_candidates_;
  ros::Time last_stamp_;
  ros::Time init_timestamp_;

  // Constants.
  const uint8_t max_buffer;
  const uint8_t match_threshold;
  const double msgs_delay_threshold;

  // Image numbers and initialization.
  uint8_t good_matches_;
  uint64_t last_image_number_;
  int64_t offset_;
  bool initialized_;
  bool first_time_;

  // Configuration.
  ros::Duration imu_offset_;

  std::mutex mutex_;
};
} // namespace synchronizer_ros

#endif // SYNCHRONIZER_DVL_H_
