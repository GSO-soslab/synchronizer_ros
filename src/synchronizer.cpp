////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  synchronizer_ros_synchronizer.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  Used to merge image time stamps and image data.
//
////////////////////////////////////////////////////////////////////////////////

#include "synchronizer_ros/synchronizer.h"

namespace synchronizer_ros {

Synchronizer::Synchronizer(const ros::NodeHandle &nh,
                           const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), image_transport_(nh),
      max_buffer(10), first_time_(true),
      match_threshold(4), msgs_delay_threshold(0.1),
      good_matches_(0), initialized_(false), device_name_("NO")
{
  ROS_INFO("[synchronizer-%s]: !!!!! Remap image time started !!!!!", device_name_.c_str());
  readParameters();

/******************** Subscriber ********************/

  // Subscriber of the non-corrected image directly from the ROS camera driver.
  image_sub_ = nh_.subscribe(camera_topic_, 10u, &Synchronizer::imageCallback, this);
  ROS_INFO("[synchronizer-%s]:   Subscribing to %s.", device_name_.c_str(), camera_topic_.c_str());

  // Subscriber for the image time message containing both the corrected
  // timestamp and the sequence number.
  image_time_sub_ = nh_.subscribe(image_time_sub_topic_, 10u,
                    &Synchronizer::imageTimeCallback, this);
  ROS_INFO("[synchronizer-%s]:   Subscribing to %s.", device_name_.c_str(), image_time_sub_topic_.c_str());

/******************** Publihser ********************/
  // Publisher to let the triggering board know as soon as the camera is
  // initialized.
  initialized_pub_ = nh_.advertise<std_msgs::Bool>(initialized_pub_topic_, 1u);
  ROS_INFO("[synchronizer-%s]:   Publishing initialization status to %s.", device_name_.c_str(), initialized_pub_topic_.c_str());

  image_fast_pub_ = image_transport_.advertise(image_pub_topic_, 10u);
  ROS_INFO("[synchronizer-%s]:   Publishing image to %s.", device_name_.c_str(), image_pub_topic_.c_str());
}

Synchronizer::~Synchronizer() {
  image_time_stamp_candidates_.clear();
  image_candidates_.clear();
  nh_.shutdown();
}

void Synchronizer::associateTimeStampsAndCleanUp() {
  if (image_candidates_.empty() || image_time_stamp_candidates_.empty()) {
    return;
  }
  // Association and cleanup.
  uint64_t last_time_num = 0u;
  uint64_t last_img_num = 0u;
  auto image_idx = image_candidates_.begin();
  auto image_time_idx = image_time_stamp_candidates_.begin();
  bool image_found = false;

  //// maybe change to better logic ?
  while (image_idx != image_candidates_.end()) {
    while (image_time_idx != image_time_stamp_candidates_.end()) {
      if (image_idx->number == image_time_idx->number + offset_) {
        //// assign trigger timestamp
        image_idx->image.header.stamp = image_time_idx->time + imu_offset_;
        //// TODO: assgin exposure time
        if(device_name_=="right_cam")
          image_idx->exposure = image_time_idx->exposure_pri;
        else if(device_name_=="left_cam")
          image_idx->exposure = image_time_idx->exposure_sec;
        // ROS_INFO("[synchronizer-%s]: exposure %f.", device_name_.c_str(), image_idx->exposure);
        
        publishImg(*image_idx);

        last_img_num = image_idx->number;
        last_time_num = image_time_idx->number;

        //// earse all matched image and time
        image_idx = image_candidates_.erase(image_idx);
        image_time_idx = image_time_stamp_candidates_.erase(image_time_idx);
        
        image_found = true;
        break;
      } 
      else {
        ++image_time_idx;
      }
    }
    //// set image time to the begin, then re-start match for new image
    image_time_idx = image_time_stamp_candidates_.begin();
    if (!image_found) 
      //// not found, go to next image,
      //// if found, it is erased, so no need to increase
      ++image_idx;
    else 
      image_found = false;
  }

  // Delete old messages to prevent non-consecutive publishing.
  image_idx = image_candidates_.begin();
  image_time_idx = image_time_stamp_candidates_.begin();
  while (image_idx != image_candidates_.end()) {
    if (image_idx->number <= last_img_num) {
      ROS_WARN("[synchronizer-%s]: Deleted old image message num. %ld, last num. %ld", 
                device_name_.c_str(),image_idx->number, last_img_num);
      image_idx = image_candidates_.erase(image_idx);
    } else {
      ++image_idx;
    }
  }
  while (image_time_idx != image_time_stamp_candidates_.end()) {
    if (image_time_idx->number <= last_time_num) {
      ROS_WARN("[synchronizer-%s]: Deleted old image time message num %ld.", 
                device_name_.c_str(), image_time_idx->number + offset_);
      image_time_idx = image_time_stamp_candidates_.erase(image_time_idx);
    } else {
      ++image_time_idx;
    }
  }
}

void Synchronizer::imageCallback( const image_numbered_msgs::ImageNumbered &image_msg) {
  std::lock_guard<std::mutex> mutex_lock(mutex_);

  if (initialized_) {
    image_candidates_.emplace_back(image_msg);
    associateTimeStampsAndCleanUp();
    if (image_candidates_.size() > max_buffer) {
      image_candidates_.erase(image_candidates_.begin());
      ROS_WARN("[synchronizer-%s]: Image candidates buffer overflow at %ld.", device_name_.c_str(), image_msg.number);
    }
  } 
  else {
    // Initialization procedure.
    ROS_INFO_ONCE("[synchronizer-%s]: INITIALIZING...", device_name_.c_str());

    const int64_t offset = image_msg.number - init_time_.number;
    ROS_INFO("[synchronizer-%s]:   Current offset %ld with img num:%ld, time num:%ld; time offset %0.5f", 
             device_name_.c_str(), offset,
             image_msg.number, init_time_.number,
             ros::Time::now().toSec() - init_timestamp_.toSec());

    // Ensure that we match to the right image by enforcing a similar arrival
    // time and the same offset.
    // Note: The image_time message should arrive prior to the image.
    if (ros::Time::now().toSec() - init_timestamp_.toSec() < msgs_delay_threshold &&
        offset == offset_) {
      ++good_matches_;
      if (good_matches_ >= match_threshold) {
        ROS_INFO("[synchronizer-%s]: Initialized with %ld offset.", device_name_.c_str(), offset);
        initialized_ = true;
        std_msgs::Bool init_msg;
        init_msg.data = true;
        initialized_pub_.publish(init_msg);
      }
    } 
    else {
      good_matches_ = 0;
    }
    offset_ = offset;
  }
}

void Synchronizer::imageTimeCallback( const synchronizer_ros::TimeNumbered &image_triggered_time_msg) {
  
  std::lock_guard<std::mutex> mutex_lock(mutex_);
  ROS_INFO_ONCE("[synchronizer-%s]: Received first image time stamp message.",device_name_.c_str());


  if (initialized_) {
    image_time_stamp_candidates_.emplace_back(image_triggered_time_msg);
    associateTimeStampsAndCleanUp();
    if (image_time_stamp_candidates_.size() > max_buffer) {
      image_time_stamp_candidates_.erase(image_time_stamp_candidates_.begin());
      ROS_WARN("[synchronizer-%s]: Time candidates buffer overflow at %ld.",
                device_name_.c_str(), image_triggered_time_msg.number);
    }
  } 
  else {
    init_timestamp_ = ros::Time::now();
    init_time_ = image_triggered_time_msg;
  }
}

void Synchronizer::publishImg( const image_numbered_msgs::ImageNumbered &image_msg) {
  if (image_msg.image.header.stamp.toSec() == 0) {
    ROS_WARN("[synchronizer-%s]: Zero timestamp for num. %ld.", device_name_.c_str(), image_msg.number);
    return;
  }

  if (image_msg.image.header.stamp.toSec() <= last_stamp_.toSec()) {
    ROS_WARN("[synchronizer-%s]: Non-increasing timestamp for num. %ld.", device_name_.c_str(), image_msg.number);
    return;
  }
  //// remind some images are deleted, except first time
  if (image_msg.number > last_image_number_ + 1) {
    if(first_time_)
      first_time_ = false;
    else
      ROS_WARN("[synchronizer-%s]: Skipped %ld frame for unknown reasons, this img num:%ld, last img num:%ld.", 
                device_name_.c_str(), 
                image_msg.number - 1 - last_image_number_,
                image_msg.number, last_image_number_);
  }
  last_image_number_ = image_msg.number;
  last_stamp_ = image_msg.image.header.stamp;


  image_fast_pub_.publish(image_msg.image);

}

bool Synchronizer::readParameters() {
  ROS_INFO("[synchronizer-%s]: SETTINGS: ", device_name_.c_str() );

/******************** camera driver side ********************/ 

  if (!nh_private_.getParam("camera_topic", camera_topic_)) {
    ROS_ERROR("[synchronizer-%s]: Define an image topic from the camera driver.", device_name_.c_str());
  } 
  else {
    if (camera_topic_.back() != '/') 
      camera_topic_ = camera_topic_ + "/";

    camera_topic_ = camera_topic_ + "image_numbered";

    image_pub_topic_ = camera_topic_ + "/image_raw_sync";
  }

/******************** synchronizer_arduino side ********************/ 

  if (!nh_private_.getParam("synchronizer_topic", synchronizer_topic_)) {
    ROS_ERROR("[synchronizer-%s]: Define a topic range where the corrected image is published.",device_name_.c_str());
  } 
  else {
    if (synchronizer_topic_.back() != '/') {
      synchronizer_topic_ = synchronizer_topic_ + "/";
    }
    image_time_sub_topic_ = synchronizer_topic_ + "image_time";
    initialized_pub_topic_ = synchronizer_topic_ + "init";
  }
 

  if (!nh_private_.getParam("device_name", device_name_)) {
    ROS_ERROR("[synchronizer-%s]: Define a device name.", device_name_.c_str());
  } 

/******************** configuration side ********************/ 

  int imu_offset_us;
  nh_private_.param("imu_offset_us", imu_offset_us, 0);
  imu_offset_ = ros::Duration(imu_offset_us / 1e6);
  ROS_INFO("[synchronizer-%s]:   IMU has a calibrated dalay of %0.0f us.",
            device_name_.c_str(), imu_offset_.toSec() * 1e6);
  return true;
}

} // namespace synchronizer_ros
