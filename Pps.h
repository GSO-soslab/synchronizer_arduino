#ifndef PPS_H_
#define PPS_H_

#include "Arduino.h"
#include <ros.h>
#include <sensor_msgs/TimeReference.h>
#include <time.h>
#include "configuration.h"


class Pps {
public:
  Pps(ros::NodeHandle *nh, const String &topic, const uint8_t trigger_pin);

  void begin();

  void setTimeNow(bool utc_clock, uint32_t curr_time_base, uint32_t start_time);

  void setNotAvailable();
  
  bool isAvailable() { return available_; };

  void publishTimeROS();

  uint32_t getOffset() { return micro_offset_; };

  char * getGPGGA() { return gpgga; };
  char * getGPGSA() { return gpgsa; };
  char * getGPGSV1() { return gpgsv_1; };
  char * getGPGSV2() { return gpgsv_2; };
  char * getGPRMC() { return gprmc; };
  char * getGPZDA() { return gpzda; };

private:

  void encodeTimeNMEA(uint32_t curr_time);

  void encodeTimeROS(uint32_t curr_time);

  // hardware pin in arduino to generate pps
  const uint8_t trigger_pin_;
  String topic_;
  uint32_t micro_offset_;
  // ROS
  ros::NodeHandle *nh_;
  ros::Publisher publisher_;
  String pub_topic_;

  // pps time used in Jetson
  char gpgga[100];
  char gpgsa[100];
  char gpgsv_1[100];
  char gpgsv_2[100];
  char gprmc[100];
  char gpzda[100];

  // pps time used in microstrain ahrs for tims sync
  sensor_msgs::TimeReference pps_time_msg_;
  volatile bool available_;
};

#endif  // PPS_H_
