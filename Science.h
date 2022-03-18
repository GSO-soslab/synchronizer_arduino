#ifndef SCIENCE_H_
#define SCIENCE_H_
//// Arduino
#include "Arduino.h"
//// ROS
#include <ros.h>
#include <std_msgs/String.h>
//// Customized 
#include "configuration.h"

class Science {

public:
  Science(ros::NodeHandle *nh, const String &topic,  HardwareSerial* serial);

  void begin();

  void communicate();

  void publish(const char* msg);

  void setClock(bool utc_clock, uint32_t start_time, uint32_t curr_time_base);

private:

  //// ROS
  ros::NodeHandle *nh_;
  ros::Publisher publisher_info_;
  String topic_info_;
  std_msgs::String msg_info_;

  //// Serial 
  Stream* serial_;
  String str_received_;

  //// Clock 
  volatile bool utc_clock_;
  volatile uint32_t start_time_;
  volatile uint32_t curr_time_base_;

  volatile bool received_;

  uint32_t time_prev_;

};

#endif  // SCIENCE_H_
