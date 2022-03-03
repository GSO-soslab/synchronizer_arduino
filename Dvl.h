#ifndef DVL_H_
#define DVL_H_

#include "Arduino.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <time.h>
#include "configuration.h"
#include <synchronizer_ros/TimeNumbered.h>


class Dvl {
public:
  Dvl(ros::NodeHandle *nh, const String &topic, const uint8_t trigger_pin);

  void initialize();

  void begin();

  void setTimeNow();

  void setNotAvailable();
  
  bool isAvailable() { return available_; };

  void encodeTimeROS(uint32_t curr_sec, uint32_t curr_micro);

  void publishTimeROS();

  uint32_t getTime() { return time_; };

  void setClock(bool utc_clock, uint32_t start_time, uint32_t curr_time_base);

  void publish();

  void initCallback(const std_msgs::Bool &msg);

private:

  // hardware pin in arduino to generate Dvl
  const uint8_t trigger_pin_;
  uint32_t time_;
  uint32_t msg_number_;
  volatile bool available_;

  uint8_t initialized_sensors_;
  bool initialized_;
  
  // ROS
  ros::NodeHandle *nh_;

  ros::Publisher publisher_time_;
  ros::Publisher publisher_info_;

  ros::Subscriber<std_msgs::Bool, Dvl> subscriber_init_;

  String topic_time_;
  String topic_info_;

  synchronizer_ros::TimeNumbered msg_time_;
  std_msgs::String msg_info_;

  //// Clock 
  volatile bool utc_clock_;
  volatile uint32_t start_time_;
  volatile uint32_t curr_time_base_;
};

#endif  // DVL_H_
