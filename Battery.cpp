#include "Battery.h"

Battery::Battery(ros::NodeHandle *nh, const String &topic) 
  : nh_(nh), 
    topic_info_(topic+"info"), publisher_info_(topic_info_.c_str(), &msg_info_),
    topic_battery_(topic+"battery"), publisher_battery_(topic_battery_.c_str(), &msg_battery_),
    time_prev_(0), battery_size_(0)
{
  publisher_info_ = ros::Publisher("/rov/synchronizer/battery/info", &msg_info_);
  publisher_battery_ = ros::Publisher("/rov/synchronizer/battery/battery", &msg_battery_);

  nh_->advertise(publisher_info_);
  nh_->advertise(publisher_battery_);
}

void Battery::measurement() {

  //// get measurements every 1s
  uint32_t time_curr = micros();
  if(time_curr - time_prev_ > BATTERY_MEASURE_TIME) {
    //// read analog measurements
    int currentValue = analogRead(BATTERY_CURRENT_PIN);
    int voltageValue = analogRead(BATTERY_VOLTAGE_PIN);
    //// store actual measurements
    battery_current_[battery_size_] = (currentValue * (3.3/1024.0)-0.33)*38.8788;
    battery_voltage_[battery_size_] = voltageValue * (3.3/1024.0)* 12.0;
    battery_size_ ++;
    //// store current time
    time_prev_ = micros();
  }
  
  //// publish measurements every 5s
  if(battery_size_ == BATTERY_PUB_SIZE) {
    //// get stored measurements
    float avg_current = 0;
    float avg_voltage = 0;
    for(int i=0; i<BATTERY_PUB_SIZE; i++) {
      avg_current += battery_current_[i];
      avg_voltage += battery_voltage_[i];
    }
    //// clean the store buffer
    battery_size_ = 0;
    memset(battery_current_, 0, sizeof(battery_current_));
    memset(battery_voltage_, 0, sizeof(battery_voltage_));

    //// put into ROS msg
    float measurement[2];
    measurement[0] = avg_current/BATTERY_PUB_SIZE;
    measurement[1] = avg_voltage/BATTERY_PUB_SIZE;
    msg_battery_.data = measurement;
    msg_battery_.data_length = 2; 
    //// publish
    publisher_battery_.publish( &msg_battery_ );

    // //// sending warning:
    if ( measurement[1] <= BATTERY_WARNING) {
      String vol = String(measurement[1]);
      String err = String("W: dangerous battery " + vol);
      msg_info_.data = String(err).c_str();
      publisher_info_.publish(&msg_info_);
    }
  }

}