#include "synchronizer_ros/synchronizer_dvl.h"


namespace synchronizer_ros {

SynchronizerDvl::SynchronizerDvl(const ros::NodeHandle &nh, 
                                 const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), device_name_("NO"), 
      received_cp_(false), received_bt_(false), initialized_(false),
      offset_(0), good_matches_(0)
{
/******************** Configuration ********************/
  readParameters();

/******************** Subscriber ********************/
  sub_time_ = nh_.subscribe("trigger_time",    20, &SynchronizerDvl::timeCallback,           this);
  sub_bt_   = nh_.subscribe("bottom_track",    20, &SynchronizerDvl::bottomtrackCallback,    this);
  sub_cp_   = nh_.subscribe("current_profile", 20, &SynchronizerDvl::currentprofileCallback, this);

  ROS_INFO("[synchronizer-%s]: Subscribing `time` to %s", device_name_.c_str(), sub_time_.getTopic().c_str());
  ROS_INFO("[synchronizer-%s]: Subscribing `bottom track` to %s", device_name_.c_str(), sub_bt_.getTopic().c_str());
  ROS_INFO("[synchronizer-%s]: Subscribing `current profile` to %s", device_name_.c_str(), sub_cp_.getTopic().c_str());

/******************** Publihser raw data ********************/
  pub_init_ = nh_.advertise<std_msgs::Bool>("init", 1);
  pub_bt_   = nh_.advertise<ds_sensor_msgs::NortekDF21>("bottom_track_sync", 20);
  pub_cp_   = nh_.advertise<ds_sensor_msgs::NortekDF3>("current_profile_sync", 20);

  ROS_INFO("[synchronizer-%s]: Publishing `initialization` to %s.", device_name_.c_str(), pub_init_.getTopic().c_str());
  ROS_INFO("[synchronizer-%s]: Publishing `synchronized bottom track` to %s.", device_name_.c_str(), pub_bt_.getTopic().c_str());
  ROS_INFO("[synchronizer-%s]: Publishing `synchronized current profile` to %s.", device_name_.c_str(), pub_cp_.getTopic().c_str());

/******************** Publihser derived data ********************/
  if(pub_derived_msg_) {
    pub_dvl_        = nh_.advertise<ds_sensor_msgs::Dvl>("dvl", 20);
    pub_velocity_   = nh_.advertise<geometry_msgs::TwistStamped>("velocity", 20);
    pub_depth_      = nh_.advertise<geometry_msgs::PointStamped>("depth", 20);
    pub_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud", 20);
  }

}

SynchronizerDvl::~SynchronizerDvl() {
  candidates_time_.clear();
  candidates_bt_.clear();
  candidates_cp_.clear();
  nh_.shutdown();
}

void SynchronizerDvl::derivedCP(geometry_msgs::PointStamped* depth_data, 
                                ds_sensor_msgs::NortekDF3* big_msg) {
  /******************** Depth data ********************/
  depth_data->header = big_msg->header;
  depth_data->point.x = 0.0;
  depth_data->point.y = 0.0;
  depth_data->point.z = big_msg->pressure;
}

void SynchronizerDvl::derivedBT(ds_sensor_msgs::Dvl* dvl_data, 
                                geometry_msgs::TwistStamped* velocity_data,
                                geometry_msgs::PointStamped* depth_data, 
                                sensor_msgs::PointCloud2& pointcloud_data,
                                ds_sensor_msgs::NortekDF21* big_msg)
{
  /******************** DVL ********************/
  /*========== Header ==========*/
  dvl_data->header = big_msg->header; 
  dvl_data->ds_header = big_msg->ds_header; 
  dvl_data->dvl_time = big_msg->dvl_time;

  /*========== Setting ==========*/
  // type
  dvl_data->dvl_type = ds_sensor_msgs::Dvl::DVL_TYPE_PISTON;
  // velocity mode, Bottom tracking or Water tracking 
  if(big_msg->version == 1)
      dvl_data->velocity_mode = dvl_data->DVL_MODE_BOTTOM;  
  else
      dvl_data->velocity_mode = dvl_data->DVL_MODE_WATER;  
  // coordinate, velocity in XYZ on DVL frame
  dvl_data->coordinate_mode = dvl_data->DVL_COORD_INSTRUMENT;

  /*========== Data ==========*/
  // fill in velocity and quality (FOM - measurement white noise level)
  dvl_data->velocity.x = big_msg->velX;
  dvl_data->velocity.y = big_msg->velY;
  dvl_data->quality.x = big_msg->fomX;
  dvl_data->quality.y = big_msg->fomY;

  if (big_msg->velZ1 == -32.768f && big_msg->velZ2 != -32.768f) {
      dvl_data->velocity.z = big_msg->velZ2;
      dvl_data->quality.z = big_msg->fomZ2;
  }
  else if (big_msg->velZ1 != -32.768f && big_msg->velZ2 == -32.768f){
      dvl_data->velocity.z = big_msg->velZ1;
      dvl_data->quality.z = big_msg->fomZ1;
  }
  else {
      dvl_data->velocity.z = (big_msg->velZ1 + big_msg->velZ2)/ 2.0;
      dvl_data->quality.z = (big_msg->fomZ1 + big_msg->fomZ2) / 2.0;
  }

  // range: convert vertical distance to range that from beam to target
  dvl_data->range[0] = big_msg->distBeam[0]  / cos(beam_angle_ * PI / 180.0);
  dvl_data->range[1] = big_msg->distBeam[1]  / cos(beam_angle_ * PI / 180.0);
  dvl_data->range[2] = big_msg->distBeam[2]  / cos(beam_angle_ * PI / 180.0);
  dvl_data->range[3] = big_msg->distBeam[3]  / cos(beam_angle_ * PI / 180.0);

  // average distance from DVL to the surface, maybe seafloor or lower ice surface 
  dvl_data->avg_altitude = big_msg->altitude_sum / big_msg->good_beams;
  
  // the vehicle speed
  dvl_data->speed_gnd = big_msg->speed_gnd;
  dvl_data->course_gnd = big_msg->course_gnd;

  /*========== Property ==========*/
  // good beams
  dvl_data->num_good_beams = big_msg->good_beams;

  // will used for sound speed correction
  dvl_data->speed_sound = big_msg->speed_sound;
  dvl_data->temperature = big_msg->temperature;

  /******************** Velocity data ********************/
  velocity_data->header = dvl_data->header;
  velocity_data->twist.linear.x = dvl_data->velocity.x;
  velocity_data->twist.linear.y = dvl_data->velocity.y;
  velocity_data->twist.linear.z = dvl_data->velocity.z;
  
  /******************** Depth data ********************/
  depth_data->header = dvl_data->header;
  depth_data->point.x = 0.0;
  depth_data->point.y = 0.0;
  depth_data->point.z = big_msg->pressure;

  /**************************** Handle pointcloud ***********************/
  pointcloud_data.header = dvl_data->header;

  // setup the pointcloud for 4 points with only XYZ property
  sensor_msgs::PointCloud2Modifier modifier(pointcloud_data);
  modifier.setPointCloud2FieldsByString(1, "xyz");    
  modifier.resize(4); 

  // re-generate 3D location of target point
  std::vector<Eigen::Vector3d> points;
  double beam_azimuth[] = {PI/4.0, -PI/4.0, -3.0*PI/4.0, 3.0*PI/4.0};
  for (int i = 0; i < 4; i++) {
      Eigen::Vector3d pt;
      pt(0) =  big_msg->distBeam[i] * tan(beam_angle_ * PI / 180.0) * cos(beam_azimuth[i]);
      pt(1) =  big_msg->distBeam[i] * tan(beam_angle_ * PI / 180.0) * sin(beam_azimuth[i]);
      pt(2) =  big_msg->distBeam[i];
      points.push_back(pt);
  }

  // setup the points XYZ
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(pointcloud_data, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(pointcloud_data, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(pointcloud_data, "z");
  for (size_t i = 0; i < 4; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
      const Eigen::Vector3d& point = points.at(i);
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
  }
}

void SynchronizerDvl::publish(const synchronizer_ros::TimeNumbered &msg_time, 
                              ds_sensor_msgs::NortekDF21 &msg_bt) {

  //// this is beam transmit time from DVL to the bottom, using DT1_Beam
  double time_transmit = (msg_bt.timeDiff1Beam[0] + msg_bt.timeDiff1Beam[1] +
                          msg_bt.timeDiff1Beam[2] + msg_bt.timeDiff1Beam[3]) / 4;
  //// actual sync time should be: trigger time + trigger delay + half transmit time
  double time_sync = msg_time.time.toSec() + trigger_delay_ + time_transmit;
  //// send msg
  msg_bt.header.stamp.fromSec(time_sync);
  pub_bt_.publish(msg_bt);
  //// TEST:
  // printf("seq:%d, trigger time: %0.9f, delay:%f, tranmit:%0.9f, sync:%0.9f, io_time%0.9f, dvl_time%0.9f\n", 
  //       msg_bt.header.seq, msg_time.time.toSec(), trigger_delay_, time_transmit,
  //       msg_bt.header.stamp.toSec(), msg_bt.ds_header.io_time.toSec(), msg_bt.dvl_time);

  //// send derived msg
  if(pub_derived_msg_) {
    auto dvl = ds_sensor_msgs::Dvl{};
    auto velocity = geometry_msgs::TwistStamped{};
    auto depth = geometry_msgs::PointStamped{};
    auto cloud = sensor_msgs::PointCloud2{};
    derivedBT(&dvl, &velocity, &depth, cloud, &msg_bt);

    pub_dvl_.publish(dvl);
    pub_velocity_.publish(velocity);
    pub_depth_.publish(depth);
    pub_pointcloud_.publish(cloud);
  }
}

void SynchronizerDvl::publish(const synchronizer_ros::TimeNumbered &msg_time, 
                              ds_sensor_msgs::NortekDF3 &msg_cp) {
  ds_sensor_msgs::NortekDF3 msg;

  //// time_sync:  trigger time + trigger delay
  double time_sync = msg_time.time.toSec() + trigger_delay_;

  //// TODO: what is header_time inside of CP cell?

  msg_cp.header.stamp.fromSec(time_sync);
  pub_cp_.publish(msg_cp);

  ////  send derived msg
  if(pub_derived_msg_) {
    auto depth = geometry_msgs::PointStamped{};
    derivedCP(&depth, &msg_cp);
    pub_depth_.publish(depth);
  }

}

void SynchronizerDvl::associateTimeStampsAndCleanUp() {
  if (candidates_time_.empty() || candidates_bt_.empty() || candidates_cp_.empty())
    return;

  uint64_t last_time_num = 0u;
  uint64_t last_dvl_num = 0u;

  std::vector<synchronizer_ros::TimeNumbered>::iterator it_time;
  std::map<uint64_t, ds_sensor_msgs::NortekDF21>::iterator it_bt;
  std::map<uint64_t, ds_sensor_msgs::NortekDF3>::iterator it_cp;

/****** find right match *****/

  it_time = candidates_time_.begin();
  while(it_time != candidates_time_.end()) {

    auto dvl_num = it_time->number + offset_;

    //// check BT and found match
    it_bt = candidates_bt_.find(dvl_num);
    if (it_bt != candidates_bt_.end()) {
      //// publish time corrected bottom track 
      publish(*it_time, it_bt->second);
      last_time_num = it_time->number;
      last_dvl_num = dvl_num;

      //// TEST:
      // printf("BT: seq:%d, dvl_num:%ld, time_num:%ld, offset:%ld\n", 
      //       it_bt->second.header.seq, dvl_num, last_time_num, offset_);

      //// delete matched bt msg
      candidates_bt_.erase(it_bt);

      //// delete matche time msg
      it_time = candidates_time_.erase(it_time);

      continue;
    }

    //// check CP and found match
    it_cp = candidates_cp_.find(dvl_num);
    if (it_cp != candidates_cp_.end()) {
      //// publish time corrected current profile 
      publish(*it_time, it_cp->second);
      last_time_num = it_time->number;
      last_dvl_num = dvl_num;

      //// TEST:
      // printf("CP: seq:%d, dvl_num:%ld, time_num:%ld, offset:%ld\n", 
      //       it_cp->second.header.seq, dvl_num, last_time_num, offset_);

      //// delete matched bt msg
      candidates_cp_.erase(it_cp);

      //// delete matche time msg
      it_time = candidates_time_.erase(it_time);

      continue;
    }

    //// not matched
    it_time++;
  }

/****** delete late unmatched msg *****/

  //// check if unmatched time msg left  
  it_time = candidates_time_.begin();
  while (it_time != candidates_time_.end()) {
    if (it_time->number <= last_time_num) {
      ROS_WARN("[synchronizer-%s]: Deleted old time message num. %ld, last num. %ld", 
                device_name_.c_str(), it_time->number, last_time_num);
      it_time = candidates_time_.erase(it_time);
    } else {
      it_time++;
    }
  }

  //// check if unmatched BT msg left  
  it_bt = candidates_bt_.begin();
  while (it_bt != candidates_bt_.end()) {
    if (it_bt->first <= last_dvl_num) {
      ROS_WARN("[synchronizer-%s]: Deleted old BT message num. %ld, last num. %ld", 
                device_name_.c_str(), it_bt->first, last_dvl_num);
      it_bt = candidates_bt_.erase(it_bt);
    } else {
      it_bt++;
    }
  }

  //// check if unmatched CP msg left  
  it_cp = candidates_cp_.begin();
  while (it_cp != candidates_cp_.end()) {
    if (it_cp->first <= last_dvl_num) {
      ROS_WARN("[synchronizer-%s]: Deleted old CP message num. %ld, last num. %ld", 
                device_name_.c_str(), it_cp->first, last_dvl_num);
      it_cp = candidates_cp_.erase(it_cp);
    } else {
      it_cp++;
    }
  }

}

void SynchronizerDvl::bottomtrackCallback(const ds_sensor_msgs::NortekDF21 &msg_bt) {
  // std::lock_guard<std::mutex> mutex_lock(mutex_);

  received_bt_ = true;
  latest_seq_bt_ = msg_bt.header.seq;

  if(initialized_) {
    candidates_bt_.insert({latest_seq_cp_+latest_seq_bt_, msg_bt});

    //// TEST:
    // printf("BT:seq:%d, t: %0.5f\n", latest_seq_bt_, msg_bt.header.stamp.toSec());

    associateTimeStampsAndCleanUp();

    if (candidates_bt_.size() > max_buffer_) {
      candidates_bt_.erase(candidates_bt_.begin());
      ROS_WARN("[synchronizer-%s]: BT candidates buffer overflow at seq %d.",
                device_name_.c_str(), latest_seq_bt_);
    }
  }
  else if (received_bt_ && received_cp_) {
    //// initialization process only start after both BT and CP are received

    double delay = ros::Time::now().toSec() - init_timestamp_.toSec();
    const int64_t offset = latest_seq_cp_ + latest_seq_bt_ - init_time_.number;
    ROS_INFO("[synchronizer-%s]:   Current offset %ld with dvl num:%d, time num:%ld; time offset %0.5f", 
             device_name_.c_str(), offset, latest_seq_cp_ + latest_seq_bt_, init_time_.number, delay);

    //// check if constant offset exists
    //// Note: The time message should arrive prior to the DVL message(both BT and CP).
    if (delay < delay_threshold_ && offset == offset_) 
      good_matches_ ++;
    else
      good_matches_ = 0;

    //// check how many good matches exist
    if (good_matches_ >= match_threshold_) {
      initialized_ = true;

      std_msgs::Bool init_msg;
      init_msg.data = true;
      pub_init_.publish(init_msg);

      ROS_INFO("[synchronizer-%s]: Initialized with %ld offset.", device_name_.c_str(), offset);
    }
    else if (good_matches_ == 0 && offset_!=0) {
      ROS_WARN("[synchronizer-%s]:  matches broken with time offset %0.5f, num offset %ld",
                device_name_.c_str(), delay, offset_);
    }
    
    offset_ = offset;

  }
  else {
    ROS_INFO("[synchronizer-%s]: waitting for CP", device_name_.c_str());
  }

}

void SynchronizerDvl::currentprofileCallback(const ds_sensor_msgs::NortekDF3 &msg_cp) {
  // std::lock_guard<std::mutex> mutex_lock(mutex_);

  received_cp_ = true;
  latest_seq_cp_ = msg_cp.header.seq;

  if(initialized_) {
    candidates_cp_.insert({latest_seq_cp_+latest_seq_bt_, msg_cp});

    associateTimeStampsAndCleanUp();

    if (candidates_cp_.size() > max_buffer_) {
      candidates_cp_.erase(candidates_cp_.begin());
      ROS_WARN("[synchronizer-%s]: CP candidates buffer overflow at seq %d.",
                device_name_.c_str(), latest_seq_cp_);
    }

  }
  else if (received_bt_ && received_cp_) {
    //// initialization process only start after both BT and CP are received

    double delay = ros::Time::now().toSec() - init_timestamp_.toSec();
    const int64_t offset = latest_seq_cp_ + latest_seq_bt_ - init_time_.number;
    ROS_INFO("[synchronizer-%s]:   Current offset %ld with dvl num:%d, time num:%ld; time offset %0.5f", 
             device_name_.c_str(), offset, latest_seq_cp_ + latest_seq_bt_, init_time_.number, delay);

    //// check if constant offset exists
    if (delay < delay_threshold_ && offset == offset_) 
      good_matches_ ++;
    else
      good_matches_ = 0;

    //// check how many good matches exist
    if (good_matches_ >= match_threshold_) {
      initialized_ = true;

      std_msgs::Bool init_msg;
      init_msg.data = true;
      // pub_init_.publish(init_msg);

      ROS_INFO("[synchronizer-%s]: Initialized with %ld offset.", device_name_.c_str(), offset);
    }
    else if (good_matches_ == 0 && offset_!=0) {
      ROS_WARN("[synchronizer-%s]:  matches broken with time offset %0.5f, num offset %ld",
                device_name_.c_str(), delay, offset_);
    }
    
    offset_ = offset;

  }
  else {
    ROS_INFO("[synchronizer-%s]: waitting for BT", device_name_.c_str());
  }

}

void SynchronizerDvl::timeCallback( const synchronizer_ros::TimeNumbered &msg_time) {
  // std::lock_guard<std::mutex> mutex_lock(mutex_);

  if(initialized_) {
    candidates_time_.emplace_back(msg_time);

    associateTimeStampsAndCleanUp();

    if (candidates_time_.size() > max_buffer_) {
      candidates_time_.erase(candidates_time_.begin());
      ROS_WARN("[synchronizer-%s]: Time candidates buffer overflow at %ld.",
                device_name_.c_str(), msg_time.number);
    }
  }
  else {
    init_timestamp_ = ros::Time::now();
    init_time_ = msg_time;
  }

}

bool SynchronizerDvl::readParameters() {

  if (!nh_private_.getParam("device_name", device_name_))
    ROS_ERROR("[synchronizer-%s]: Define a device name.", device_name_.c_str());

  nh_private_.param<double>("sound_speed", sound_speed_, 1500);
  nh_private_.param<double>("trigger_delay", trigger_delay_, 0.0084);
  nh_private_.param<double>("delay_threshold", delay_threshold_, 0.15);
  nh_private_.param<int>("match_threshold", match_threshold_, 5);
  nh_private_.param<int>("max_buffer", max_buffer_, 10);
  nh_private_.param<int>("beam_angle", beam_angle_, 25);
  nh_private_.param<bool>("pub_derived_msg", pub_derived_msg_, false);



  ROS_INFO("[synchronizer-%s]: SETTINGS: sound speed:%f, trigger delay ms:%f, pub_derived_msg:%s", 
           device_name_.c_str(), sound_speed_, trigger_delay_, pub_derived_msg_ ? "true":"false");

  return true;
}

} // namespace synchronizer_ros

int main(int argc, char **argv) {
  ros::init(argc, argv, "synchronizer_dvl_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  synchronizer_ros::SynchronizerDvl dvl_node(nh,nh_private);

  ros::spin();

  return 0;
}