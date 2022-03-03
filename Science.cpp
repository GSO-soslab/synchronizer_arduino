#include "Science.h"

Science::Science(ros::NodeHandle *nh, const String &topic, HardwareSerial* serial) 
  : nh_(nh), topic_info_(topic+"info"), publisher_info_(topic_info_.c_str(), &msg_info_),
    curr_time_base_(0), utc_clock_(false), start_time_(0), received_(false)
{
  publisher_info_ = ros::Publisher("/rov/synchronizer/science/info", &msg_info_);
  nh_->advertise(publisher_info_);

  serial_ = serial;
}


void Science::setClock(bool utc_clock, uint32_t start_time, uint32_t curr_time_base) {
  utc_clock_      = utc_clock;
  start_time_     = start_time;
  curr_time_base_ = curr_time_base;
}

void Science::receive() {

  //// append received every byte
  while(serial_->available()) {
    str_received_ += char(serial_->read());
  }
  
  //// a line is received, which means '\n' is found
  // if (str_received_.lastIndexOf('\n') > 0 && !received_) {
  //   msg_info_.data = str_received_.c_str();
  //   publisher_info_.publish(&msg_info_);

  //   str_received_="";
  //   received_ = true;
  // }  

  if (str_received_.lastIndexOf('\n') > 0) {

    msg_info_.data = str_received_.c_str();
    publisher_info_.publish(&msg_info_);

    str_received_="";
  }  

  //// TODO: handle message from science system, no need to send all heartbeat all the times
}

void Science::publish(const char* msg) {

  // check size, start and end delimiters: #id*
  if(strlen(msg) == 3 && msg[0] == '#' && msg[2] == '*') {

    switch (msg[1]) {
      // stop record
      case '0': {
        serial_->write("$0,*\n",6);
        received_ = false;

        break;
      }

      // start record
      case '1': {
        received_ = false;

        // come up with clock
        uint32_t time_aft_start, latest_sec;
        time_aft_start = micros() - start_time_;

        if(utc_clock_)
          latest_sec = curr_time_base_ + time_aft_start / 1000000;
        else
          latest_sec = TIME_BASE + time_aft_start / 1000000;

        // get ready message
        String str_time = String(latest_sec);
        String str_all = String("$1," + str_time + ",7,*\n");

        // send
        serial_->write(str_all.c_str());
        break;
      }

      // print saved file path
      case '2': {
        received_ = false;

        serial_->write("$2,*\n", 6);
        break;
      }

      // display latest sensor data
      case '3': {
        received_ = false;

        serial_->write("$3,*\n", 6);
        break;
      }


      default:
        break;
    }
  }

}
