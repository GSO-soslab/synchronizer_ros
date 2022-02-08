////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  versavis_synchronizer_nodelet.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  Nodelet wrapper for versavis_synchronizer.
//
////////////////////////////////////////////////////////////////////////////////

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "synchronizer_ros/synchronizer.h"

namespace synchronizer_ros {

class SynchronizerNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      synchronizer = std::make_shared<Synchronizer>(
          getNodeHandle(), getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<Synchronizer> synchronizer;
};
} // namespace synchronizer_ros

PLUGINLIB_EXPORT_CLASS(synchronizer_ros::SynchronizerNodelet, nodelet::Nodelet);
