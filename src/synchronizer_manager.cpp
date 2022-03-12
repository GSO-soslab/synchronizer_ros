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

#include <chrono>
#include <thread>

class SynchronizerManager {

public:
  SynchronizerManager() :
    nh_sci("~sci"), nh_led("~led"), nh_servo("~servo"), nh_clock("~clock"),
    server_sci(nh_sci), server_led(nh_led), server_servo(nh_servo), server_clock(nh_clock)
    {

    // pub to arduino
    pub_sciTask = nh.advertise<std_msgs::String>("/rov/synchronizer/science/task", 1);
    pub_led = nh.advertise<std_msgs::String>("/rov/synchronizer/led/cmd", 1);

    pub_servoCmd = nh.advertise<std_msgs::UInt16>("/rov/synchronizer/servo/cmd", 1);
    pub_clock = nh.advertise<std_msgs::UInt32>("/rov/synchronizer/reset_clock", 1);

    // sub from arduino
    sub_sys = nh.subscribe<std_msgs::String>("/rov/synchronizer/system", 1, 
                                             &SynchronizerManager::systemCallback, this);
    sub_sci = nh.subscribe<std_msgs::String>("/rov/synchronizer/science/info", 1, 
                                             &SynchronizerManager::scienceCallback, this);
    sub_battery = nh.subscribe<std_msgs::String>("/rov/synchronizer/battery/info", 1, 
                                             &SynchronizerManager::batteryCallback, this);
    sub_cam = nh.subscribe<std_msgs::String>("/rov/synchronizer/cam/info", 1, 
                                             &SynchronizerManager::cameraCallback, this);
    sub_dvl = nh.subscribe<std_msgs::String>("/rov/synchronizer/dvl/info", 1, 
                                             &SynchronizerManager::dvlCallback, this);
    sub_pps = nh.subscribe<std_msgs::String>("/rov/synchronizer/pps/info", 1, 
                                             &SynchronizerManager::ppsCallback, this);

    // servers from onboard computer
    server_sci.setCallback(boost::bind(&SynchronizerManager::scienceCallback, this, _1, _2));
    server_led.setCallback(boost::bind(&SynchronizerManager::ledCallback, this, _1, _2));
    server_servo.setCallback(boost::bind(&SynchronizerManager::servoCallback, this, _1, _2));
    server_clock.setCallback(boost::bind(&SynchronizerManager::clockCallback, this, _1, _2));

    // load params 
    last_led_mode = 0;
    last_led_pwm = 1100;

    last_sci_task = 0;
    last_sci_manual = "aaa";
  }

  //// system information callbacks
  void systemCallback(const std_msgs::String::ConstPtr& msg);

  void scienceCallback(const std_msgs::String::ConstPtr& msg);

  void batteryCallback(const std_msgs::String::ConstPtr& msg);

  void cameraCallback(const std_msgs::String::ConstPtr& msg);

  void dvlCallback(const std_msgs::String::ConstPtr& msg);

  void ppsCallback(const std_msgs::String::ConstPtr& msg);

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
  ros::Publisher pub_led;
  ros::Publisher pub_servoCmd;
  ros::Publisher pub_clock;
  // sub
  ros::Subscriber sub_sys;
  ros::Subscriber sub_sci;
  ros::Subscriber sub_battery;
  ros::Subscriber sub_cam;
  ros::Subscriber sub_dvl;
  ros::Subscriber sub_pps;
  // server
  dynamic_reconfigure::Server<synchronizer_ros::ScienceConfig> server_sci;
  dynamic_reconfigure::Server<synchronizer_ros::LedConfig>     server_led;
  dynamic_reconfigure::Server<synchronizer_ros::ServoConfig>   server_servo;
  dynamic_reconfigure::Server<synchronizer_ros::ClockConfig>   server_clock;

  //// global variables for LED
  int last_led_mode;
  int last_led_pwm;

  int last_sci_task;
  std::string last_sci_manual;
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

void SynchronizerManager::cameraCallback(const std_msgs::String::ConstPtr& msg) {

  ROS_INFO("synchronizer_ros - cam: %s", msg->data.c_str());
}

void SynchronizerManager::dvlCallback(const std_msgs::String::ConstPtr& msg) {

  ROS_INFO("synchronizer_ros - dvl: %s", msg->data.c_str());
}

void SynchronizerManager::ppsCallback(const std_msgs::String::ConstPtr& msg) {

  ROS_INFO("synchronizer_ros - pps: %s", msg->data.c_str());
}

void SynchronizerManager::ledCallback(synchronizer_ros::LedConfig &config, uint32_t level)
{
  /******************** LED mode ********************/

  //// example: send led mode as Servo_mode: #0,0*
  //// example: send led mode as Flash_mode: #0,1*
  if(config.LED_Mode != last_led_mode){
    last_led_mode = config.LED_Mode;

    std::stringstream ss;
    std_msgs::String msg;

    ss << "#0," << last_led_mode <<"*";
    msg.data = ss.str();
    pub_led.publish(msg);
  }

  /******************** LED brightness control ********************/

  //// example: send pwm min brightness(off): #1,1100*
  //// example: send pwm max brightness:      #1,1900*
  if(config.LED_Brightness != last_led_pwm){
    last_led_pwm = config.LED_Brightness;

    std::stringstream ss;
    std_msgs::String msg;

    ss << "#1," << last_led_pwm <<"*";
    msg.data = ss.str();
    pub_led.publish(msg);
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

    //// computer clock (UTC)
    case 1:{

      ros::Time time_before = ros::Time::now();
      int delay = (1000000000 - time_before.nsec) ;
      std::this_thread::sleep_for(std::chrono::nanoseconds(delay));
      // printf("delay around %d ms to send a full second\n", delay/1000000);

      msg.data = time_before.sec + 1;
      pub_clock.publish(msg);

      // printf("UTC clock %d sent to Arduino\n", msg.data);
      break;
    }

    //// TODO: gps clock (UTC)
    case 2:
      msg.data = ros::Time::now().toSec();
      pub_clock.publish(msg);
      break;
      
    //// manual UTC clock
    case 3:
      msg.data = 1646073518;
      pub_clock.publish(msg);
      break;

    default:
      break;
  }
}

void SynchronizerManager::scienceCallback(synchronizer_ros::ScienceConfig &config, uint32_t level)
{

  if(last_sci_task != config.Task) {
    std_msgs::String task;

    switch (config.Task)
    {
      // stop record science sensor data
      case 0:
        task.data = "#0*";
        last_sci_task = 0;
        break;
      // start record science sensor data
      case 1:
        task.data = "#1*";
        last_sci_task = 1;
        break;
      // print saved file path
      case 2:
        task.data = "#2*";
        last_sci_task = 2;
        break;
      // print latest sensor data
      case 3:
        task.data = "#3*";
        last_sci_task = 3;
        break;
      // something wrong
      default:
        task.data = "#-1*";
        last_sci_task = -1;
        ROS_WARN("Unknow task id !!");
        break;
    }

    pub_sciTask.publish(task);
  }


  if(last_sci_manual != config.Manual) {

    std_msgs::String task;
    task.data = config.Manual;

    pub_sciTask.publish(task);

    // printf("send from onboard:%s\n", config.Manual.c_str());

    last_sci_manual = config.Manual;
  }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "synchronizer_ros_manager_node");

  SynchronizerManager  manager;

  ros::spin();

  return 0;
}