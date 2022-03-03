#ifndef CAM_H_
#define CAM_H_

#include "Arduino.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <time.h>
#include "configuration.h"
#include <synchronizer_ros/TimeNumbered.h>


class Cam {
public:
  Cam(ros::NodeHandle *nh, const String &topic, const uint8_t trigger_pin, const int rate);

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

  void exposurePrimary();

  void exposureSecondary();
private:

  // hardware pin in arduino to generate Cam
  const uint8_t trigger_pin_;

  volatile bool available_;

  uint16_t rate_hz_;

  uint8_t initialized_sensors_;
  bool initialized_;

  // store trigger information
  uint32_t time_;
  uint32_t msg_number_;
  uint32_t exposure_pri_;
  uint32_t exposure_sec_;

  // ROS
  ros::NodeHandle *nh_;

  ros::Publisher publisher_time_;
  ros::Publisher publisher_info_;

  ros::Subscriber<std_msgs::Bool, Cam> subscriber_init_;

  String topic_time_;
  String topic_info_;

  synchronizer_ros::TimeNumbered msg_time_;
  std_msgs::String msg_info_;

  //// Clock 
  volatile bool utc_clock_;
  volatile uint32_t start_time_;
  volatile uint32_t curr_time_base_;
};

#endif  // CAM_H_
