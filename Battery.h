#ifndef BATTERY_H_
#define BATTERY_H_
//// Arduino
#include "Arduino.h"
//// ROS
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
//// Customized 
#include "configuration.h"

class Battery {

public:
  Battery(ros::NodeHandle *nh, const String &topic);

  void measurement();

private:
  //// ROS
  ros::NodeHandle *nh_;

  ros::Publisher publisher_info_;
  ros::Publisher publisher_battery_;

  String topic_info_;
  String topic_battery_;

  std_msgs::String msg_info_;
  std_msgs::Float32MultiArray msg_battery_;

  uint32_t time_prev_;

  //// average
  float battery_voltage_[BATTERY_PUB_SIZE];
  float battery_current_[BATTERY_PUB_SIZE];
  int battery_size_;

  //// parameters
  //// TODO: add parameters to class, not directly use the parameters from configuration.h

};

#endif // BATTERY_H_