#ifndef PPS_H_
#define PPS_H_

#include "Arduino.h"
#include <ros.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/String.h>
#include <time.h>
#include "configuration.h"


class Pps {
public:
  Pps(ros::NodeHandle *nh, const String &topic, const uint8_t trigger_pin, HardwareSerial* serial);

  void begin();

  void setTimeNow();

  void setNotAvailable();
  
  bool isAvailable() { return available_; };

  void encodeTimeGPS(uint32_t curr_time);

  void encodeTimeROS(uint32_t curr_time);

  void publishTimeROS();

  void publishTimeGPS();

  uint32_t getOffset() { return micro_offset_; };

  uint32_t getTime() { return time_; };

  char * getGPGGA() { return gpgga; };
  char * getGPGSA() { return gpgsa; };
  char * getGPGSV1() { return gpgsv_1; };
  char * getGPGSV2() { return gpgsv_2; };
  char * getGPRMC() { return gprmc; };
  char * getGPZDA() { return gpzda; };

  void publish(bool utc_clock, uint32_t curr_time_base, uint32_t start_time);

private:

  // hardware pin in arduino to generate pps
  const uint8_t trigger_pin_;
  uint32_t micro_offset_;
  uint32_t time_;

  // ROS
  ros::NodeHandle *nh_;
  ros::Publisher publisher_time_;
  ros::Publisher publisher_info_;
  String topic_time_;
  String topic_info_;
  sensor_msgs::TimeReference pps_time_msg_;
  std_msgs::String pps_info_msg_;

  // pps time used in Jetson
  Stream* serial2_;
  char gpgga[100];
  char gpgsa[100];
  char gpgsv_1[100];
  char gpgsv_2[100];
  char gprmc[100];
  char gpzda[100];

  // pps time used in microstrain ahrs for tims sync
  volatile bool available_;
};

#endif  // PPS_H_
