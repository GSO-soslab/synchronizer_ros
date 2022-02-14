#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <synchronizer_ros/ScienceConfig.h>
#include <synchronizer_ros/LedConfig.h>
#include <synchronizer_ros/ServoConfig.h>
#include <synchronizer_ros/ClockConfig.h>
#include <std_msgs/String.h> 
#include <std_msgs/UInt16.h> 
#include <std_msgs/UInt32.h> 
#include <std_msgs/Bool.h> 

class SynchronizerManager {

public:
  SynchronizerManager() :
    nh_sci("~sci"), nh_led("~led"), nh_servo("~servo"), nh_clock("~clock"),
    server_sci(nh_sci), server_led(nh_led), server_servo(nh_servo), server_clock(nh_clock)
    {

    // pub to arduino
    pub_sciTask = nh.advertise<std_msgs::String>("/rov/synchronizer/science/task", 1);
    pub_ledCmd = nh.advertise<std_msgs::UInt16>("/rov/synchronizer/led/cmd", 1);
    pub_ledMode = nh.advertise<std_msgs::Bool>("/rov/synchronizer/led/mode", 1);
    pub_servoCmd = nh.advertise<std_msgs::UInt16>("/rov/synchronizer/servo/cmd", 1);
    pub_clock = nh.advertise<std_msgs::UInt32>("/rov/synchronizer/reset_clock", 1);

    // sub from arduino
    sub_sys = nh.subscribe<std_msgs::String>("/rov/synchronizer/system", 1, 
                                             &SynchronizerManager::systemCallback, this);
    sub_sci = nh.subscribe<std_msgs::String>("/rov/synchronizer/science/info", 1, 
                                             &SynchronizerManager::scienceCallback, this);
    sub_battery = nh.subscribe<std_msgs::String>("/rov/synchronizer/battery/info", 1, 
                                             &SynchronizerManager::batteryCallback, this);
    //// TODO: sub_cam_info
    //// TODO: sub_battery_info

    // servers from onboard computer
    server_sci.setCallback(boost::bind(&SynchronizerManager::scienceCallback, this, _1, _2));
    server_led.setCallback(boost::bind(&SynchronizerManager::ledCallback, this, _1, _2));
    server_servo.setCallback(boost::bind(&SynchronizerManager::servoCallback, this, _1, _2));
    server_clock.setCallback(boost::bind(&SynchronizerManager::clockCallback, this, _1, _2));

    // load params 
    last_led_cmd.data = 50;
    last_led_mode.data = false;
  }

  //// system information callbacks
  void systemCallback(const std_msgs::String::ConstPtr& msg);

  void scienceCallback(const std_msgs::String::ConstPtr& msg);

  void batteryCallback(const std_msgs::String::ConstPtr& msg);

  //// srv callbacks
  void scienceCallback(synchronizer_ros::ScienceConfig &config, uint32_t level);

  void ledCallback(synchronizer_ros::LedConfig &config, uint32_t level);

  void servoCallback(synchronizer_ros::ServoConfig &config, uint32_t level);

  void clockCallback(synchronizer_ros::ClockConfig &config, uint32_t level);

private:
  // nodehandle
  ros::NodeHandle nh;
  ros::NodeHandle nh_sci;
  ros::NodeHandle nh_led;
  ros::NodeHandle nh_servo;
  ros::NodeHandle nh_clock;
  // pub
  ros::Publisher pub_sciTask;
  ros::Publisher pub_ledCmd;
  ros::Publisher pub_ledMode;
  ros::Publisher pub_servoCmd;
  ros::Publisher pub_clock;
  // sub
  ros::Subscriber sub_sys;
  ros::Subscriber sub_sci;
  ros::Subscriber sub_battery;
  // server
  dynamic_reconfigure::Server<synchronizer_ros::ScienceConfig> server_sci;
  dynamic_reconfigure::Server<synchronizer_ros::LedConfig>     server_led;
  dynamic_reconfigure::Server<synchronizer_ros::ServoConfig>   server_servo;
  dynamic_reconfigure::Server<synchronizer_ros::ClockConfig>   server_clock;
  // global variables
  std_msgs::UInt16 last_led_cmd;
  std_msgs::Bool last_led_mode;

};

void SynchronizerManager::systemCallback(const std_msgs::String::ConstPtr& msg) {
  //// TODO: check msg 
  ROS_INFO("synchronizer_ros - system: %s", msg->data.c_str());
}

void SynchronizerManager::scienceCallback(const std_msgs::String::ConstPtr& msg) {

  ROS_INFO("synchronizer_ros - science: %s", msg->data.c_str());
}

void SynchronizerManager::batteryCallback(const std_msgs::String::ConstPtr& msg) {

  ROS_INFO("synchronizer_ros - battery: %s", msg->data.c_str());
}


void SynchronizerManager::ledCallback(synchronizer_ros::LedConfig &config, uint32_t level)
{
  if(config.LED_Brightness != last_led_cmd.data){
    last_led_cmd.data = config.LED_Brightness;
    pub_ledCmd.publish(last_led_cmd);
  }

  if(config.LED_Mode != last_led_mode.data){
    last_led_mode.data = config.LED_Mode;
    pub_ledMode.publish(last_led_mode);
  }
}

void SynchronizerManager::servoCallback(synchronizer_ros::ServoConfig &config, uint32_t level)
{
  std_msgs::UInt16 msg;
  msg.data = config.Servo_Position;
  pub_servoCmd.publish(msg);
}

void SynchronizerManager::clockCallback(synchronizer_ros::ClockConfig &config, uint32_t level)
{

  std_msgs::UInt32 msg;
  switch(config.Clock_Type)
  {
    //// don't set clock
    case 0:
      break;

    //// computer clock
    //// TODO: use UTC time?
    case 1:
      msg.data = ros::Time::now().toSec();
      pub_clock.publish(msg);
      break;

    //// gps clock
    //// TODO: gps clock
    case 2:
      msg.data = ros::Time::now().toSec();
      pub_clock.publish(msg);
      break;
      
    //// manual clock
    case 3:
      msg.data = 1642613314;
      pub_clock.publish(msg);
      break;

    default:
      break;
  }
}

void SynchronizerManager::scienceCallback(synchronizer_ros::ScienceConfig &config, uint32_t level)
{
  std_msgs::String task_id;

  switch (config.Task)
  {
    // stop record science sensor data
    case 0:
      task_id.data = "#0*";
      break;
    // start record science sensor data
    case 1:
      task_id.data = "#1*";
      break;
    // print saved file path
    case 2:
      task_id.data = "#2*";
      break;
    // print latest sensor data
    case 3:
      task_id.data = "#3*";
      break;
    // something wrong
    default:
      task_id.data = "#-1*";
      ROS_WARN("Unknow task id !!");
      break;
  }

  pub_sciTask.publish(task_id);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "synchronizer_ros_manager_node");

  SynchronizerManager  manager;

  ros::spin();

  return 0;
}