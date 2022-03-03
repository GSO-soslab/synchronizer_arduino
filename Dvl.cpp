#include "Dvl.h"



Dvl::Dvl(ros::NodeHandle *nh, const String &topic, const uint8_t trigger_pin) 
  : nh_(nh),  trigger_pin_(trigger_pin), available_(false), msg_number_(0),
    curr_time_base_(0), utc_clock_(false), start_time_(0),
    topic_time_(topic+"time"), publisher_time_(topic_time_.c_str(), &msg_time_),
    topic_info_(topic+"info"), publisher_info_(topic_info_.c_str(), &msg_info_),
    subscriber_init_((topic + "init").c_str(), &Dvl::initCallback, this)
{
  publisher_time_  = ros::Publisher("/rov/synchronizer/dvl/time", &msg_time_);
  publisher_info_  = ros::Publisher("/rov/synchronizer/dvl/info", &msg_info_);
  subscriber_init_ = ros::Subscriber<std_msgs::Bool, Dvl>(
                     "/rov/synchronizer/dvl/init", &Dvl::initCallback, this);

  nh_->advertise(publisher_time_);
  nh_->advertise(publisher_info_);
  nh_->subscribe(subscriber_init_);
}

void Dvl::initialize() {
  //// 8hz, 1ms pluse

  PORT->Group[g_APinDescription[trigger_pin_].ulPort].PINCFG[g_APinDescription[trigger_pin_].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[trigger_pin_].ulPort].PMUX[g_APinDescription[trigger_pin_].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;
  
  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;        // Configure the TCC1 timer for normal PWM mode
  while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization
  

  TCC1->PER.reg = 46874;                        // Set the TCC1 PER register to generate a 1 second period  
  while (TCC1->SYNCBUSY.bit.PER);                // Wait for synchronization
  TCC1->CC[1].reg = 94;                         // Set the TCC1 CC1 register to generate high voltage within 0.002 second (0.2%)
  while (TCC1->SYNCBUSY.bit.CC1);                // Wait for synchronization

  // NVIC_SetPriority(TCC1_IRQn, 0);                // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC1 to 0 (highest)
  // NVIC_EnableIRQ(TCC1_IRQn);                     // Connect TCC1 to Nested Vector Interrupt Controller (NVIC)

  TCC1->INTENSET.reg = TCC_INTENSET_OVF;         // Enable TCC1 overflow (OVF) interrupts
 
  TCC1->CTRLA.reg = TCC_CTRLA_PRESCSYNC_PRESC |  // Reset timer on the next prescaler clock
                    TCC_CTRLA_PRESCALER_DIV1024; // Set prescaler to 1024, 48MHz/1024 = 46875kHz                       
  
  TCC1->CTRLA.bit.ENABLE = 1;                    // Enable the TCC1 timer
  while (TCC1->SYNCBUSY.bit.ENABLE);             // Wait for synchronization  
}

void Dvl::initCallback(const std_msgs::Bool &msg) {
  // two cameras (stereo) found synchronization offsets
  if (msg.data)
    initialized_sensors_++;

  if (initialized_sensors_== 1){

    initialized_ = true;
    begin();
  }
}

void Dvl::begin() {
  TCC1->CTRLA.reg &= ~TCC_CTRLA_ENABLE;          // Disable the timers
  while (TCC1->SYNCBUSY.bit.ENABLE);

  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;        // Configure the TCC1 timer for normal PWM mode
  while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  //// set to desired trigger Hz(8)
  TCC1->PER.reg = 5859;                      // Set the TCC1 PER register to generate a 0.125 second period (12.5%)     
  while (TCC1->SYNCBUSY.bit.PER);                
  TCC1->CC[1].reg = 94;                      // Set the TCC1 CC1 register to generate high voltage within 0.002 second (0.2%)    
  while (TCC1->SYNCBUSY.bit.CC1);   

  TCC1->INTENSET.reg = TCC_INTENSET_OVF;         // Enable TCC1 overflow (OVF) interrupts
 
  TCC1->CTRLA.reg = TCC_CTRLA_PRESCSYNC_PRESC |  // Reset timer on the next prescaler clock
                    TCC_CTRLA_PRESCALER_DIV1024; // Set prescaler to 1024, 48MHz/1024 = 46875kHz                       
  
  TCC1->CTRLA.bit.ENABLE = 1;                    // Enable the TCC1 timer
  while (TCC1->SYNCBUSY.bit.ENABLE);             // Wait for synchronization   

  msg_info_.data = "dvl is initialized";
  publisher_info_.publish(&msg_info_); 
}

void Dvl::setClock(bool utc_clock, uint32_t start_time, uint32_t curr_time_base) {
  utc_clock_      = utc_clock;
  start_time_     = start_time;
  curr_time_base_ = curr_time_base;
}

void Dvl::setTimeNow() {
  if (TCC1->INTFLAG.bit.OVF && TCC1->INTENSET.bit.OVF)      // Optionally check for overflow (OVF) interrupt      
  {   
    TCC1->INTFLAG.bit.OVF = 1;                              // Clear the overflow (OVF) interrupt flag

    time_ = micros();

    msg_number_++;

    available_ = true;
  }  
}

void Dvl::setNotAvailable() {
  available_ = false;
}

void Dvl::publish() {

  if(isAvailable()) {

    //// get time duration after UTC clock is set 
    uint32_t time_aft_utc, latest_sec, latest_micro;
    time_aft_utc = time_ - start_time_;
    //// get second part
    if(utc_clock_) 
      latest_sec = curr_time_base_ + time_aft_utc / 1000000;
    else
      latest_sec = TIME_BASE + time_aft_utc / 1000000;
    //// get microssecond part
    latest_micro = time_aft_utc % 1000000;   

    //// encode time format
    encodeTimeROS(latest_sec, latest_micro);

    //// publish time 
    publishTimeROS();

    //// set not avaiable
    setNotAvailable();
  }
}

void Dvl::encodeTimeROS(uint32_t curr_sec, uint32_t curr_micro) {

  msg_time_.time = ros::Time(curr_sec, curr_micro*1000);
  msg_time_.number = msg_number_;

}

void Dvl::publishTimeROS() {
  // send time to onboard computer synchronizer_ros
  publisher_time_.publish( &msg_time_ ); 

  if(!initialized_){
    msg_info_.data = "dvl is initializing";
    publisher_info_.publish(&msg_info_); 
  }
}
