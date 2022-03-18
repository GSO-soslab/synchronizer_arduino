#include "Science.h"

Science::Science(ros::NodeHandle *nh, const String &topic, HardwareSerial* serial) 
  : nh_(nh), topic_info_(topic+"info"), publisher_info_(topic_info_.c_str(), &msg_info_),
    curr_time_base_(0), utc_clock_(false), start_time_(0), received_(false),
    time_prev_(0)
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

void Science::communicate() {
  /***** Receive msg from Science system *****/

  //// append received every byte
  while(serial_->available()) {
    str_received_ += char(serial_->read());
  }

  if (str_received_.lastIndexOf('\n') > 0) {

    msg_info_.data = str_received_.c_str();
    publisher_info_.publish(&msg_info_);

    str_received_="";
  }  

  /***** Send timestamp to Science system *****/

    //// get measurements every 1s
  uint32_t time_curr = micros();
  if(time_curr - time_prev_ > SCIENCE_SEND_TIME && utc_clock_) {

    //// get system clock
    uint32_t time_aft_utc, latest_sec, latest_msec ;
    time_aft_utc = time_curr - start_time_;
    latest_sec = curr_time_base_ + time_aft_utc / 1000000;
    latest_msec = time_aft_utc % 1000000;

    //// send to science
    String str_time = String(String(latest_sec) + '.' + String(latest_msec) + '\n');
    serial_->write(str_time.c_str());

    //// store current time
    time_prev_ = time_curr;
  }


  //// TODO: handle message from science system, no need to send all heartbeat all the times
}

void Science::publish(const char* msg) {

  //// simple task mode
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
  //// manual command mode
  else{
    
    // serial_->write(msg);
    msg_info_.data = msg;
    publisher_info_.publish(&msg_info_);

  }

}
