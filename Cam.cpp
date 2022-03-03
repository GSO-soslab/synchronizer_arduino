#include "Cam.h"

Cam::Cam(ros::NodeHandle *nh, const String &topic, const uint8_t trigger_pin, const int rate) 
  : nh_(nh),  trigger_pin_(trigger_pin), available_(false), msg_number_(0),
    curr_time_base_(0), utc_clock_(false), start_time_(0), rate_hz_(rate),
    topic_time_(topic+"time"), publisher_time_(topic_time_.c_str(), &msg_time_),
    topic_info_(topic+"info"), publisher_info_(topic_info_.c_str(), &msg_info_),
    subscriber_init_((topic + "init").c_str(), &Cam::initCallback, this),
    exposure_pri_(0), exposure_sec_(0)
{
  publisher_time_  = ros::Publisher("/rov/synchronizer/cam/time", &msg_time_);
  publisher_info_  = ros::Publisher("/rov/synchronizer/cam/info", &msg_info_);
  subscriber_init_ = ros::Subscriber<std_msgs::Bool, Cam>(
                     "/rov/synchronizer/cam/init", &Cam::initCallback, this);

  nh_->advertise(publisher_time_);
  nh_->advertise(publisher_info_);
  nh_->subscribe(subscriber_init_);
}

void Cam::initialize() {

  PORT->Group[g_APinDescription[trigger_pin_].ulPort].PINCFG[g_APinDescription[trigger_pin_].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[trigger_pin_].ulPort].PMUX[g_APinDescription[trigger_pin_].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;

  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;        // Configure the TCC0 timer for normal PWM mode
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization
  
  TCC0->PER.reg = 46874;                         // Set the TCC0 PER register to generate a 1 second period
  while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization

  TCC0->CC[1].reg = 4;                           // Set the TCC0 CC1 register to generate a duty cycle of 0.0001s
  while (TCC0->SYNCBUSY.bit.CC1);                // Wait for synchronization

  // NVIC_SetPriority(TCC0_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC0 to 0 (highest)
  // NVIC_EnableIRQ(TCC0_IRQn);         // Connect TCC0 to Nested Vector Interrupt Controller (NVIC)

  TCC0->INTENSET.reg = TCC_INTENSET_OVF;       // Enable TCC0 overflow (OVF) interrupts
 
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCSYNC_PRESC |   // Reset timer on the next prescaler clock
                    TCC_CTRLA_PRESCALER_DIV1024;  // Set prescaler to 1024, 48MHz/1024 = 46875kHz                       
  
  TCC0->CTRLA.bit.ENABLE = 1;                    // Enable the TCC0 timer
  while (TCC0->SYNCBUSY.bit.ENABLE);             // Wait for synchronization
}

void Cam::initCallback(const std_msgs::Bool &msg) {
  // two cameras (stereo) found synchronization offsets
  if (msg.data)
    initialized_sensors_++;

  if (initialized_sensors_==2){

    initialized_ = true;
    begin();
  }
}

void Cam::begin() {

  //// Disable the timers
  TCC0->CTRLA.reg &= ~TCC_CTRLA_ENABLE; 
  while (TCC0->SYNCBUSY.bit.ENABLE);
  //// Configure the TCC0 timer for normal PWM mode
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;        
  while (TCC0->SYNCBUSY.bit.WAVE);               

  switch(rate_hz_) {
    case 5: {
      //// Set the TCC0 PER register to generate a 0.2 second period (20%)
      TCC0->PER.reg = 37499;                           
      while (TCC0->SYNCBUSY.bit.PER); 
      //// Set the TCC0 CC1 register to generate high voltage within 0.0001 second (0.01%)               
      TCC0->CC[1].reg = 18;                          
      while (TCC0->SYNCBUSY.bit.CC1);  
      // // Reset timer on the next prescaler clock and Set prescaler to 256, 48MHz/256 = 187500
      TCC0->CTRLA.reg = TCC_CTRLA_PRESCSYNC_PRESC | 
                        TCC_CTRLA_PRESCALER_DIV256;   
      break;
    }

    case 10:{
      //// Set the TCC0 PER register to generate a 0.1 second period
      TCC0->PER.reg = 18749;                           
      while (TCC0->SYNCBUSY.bit.PER); 
      //// Set the TCC0 CC1 register to generate high voltage within 0.0001 second                
      TCC0->CC[1].reg = 18;                          
      while (TCC0->SYNCBUSY.bit.CC1);  
      // // Reset timer on the next prescaler clock and Set prescaler to 256, 48MHz/256 = 187500
      TCC0->CTRLA.reg = TCC_CTRLA_PRESCSYNC_PRESC | 
                        TCC_CTRLA_PRESCALER_DIV256;   
      break;
    }

    case 15:{
      //// Set the TCC0 PER register to generate a 0.066666 second period 
      TCC0->PER.reg = 49999;                           
      while (TCC0->SYNCBUSY.bit.PER); 
      //// Set the TCC0 CC1 register to generate high voltage within 0.0001 second             
      TCC0->CC[1].reg = 75;                          
      while (TCC0->SYNCBUSY.bit.CC1);  
      // // Reset timer on the next prescaler clock and Set prescaler to 256, 48MHz/64 = 750000
      TCC0->CTRLA.reg = TCC_CTRLA_PRESCSYNC_PRESC | 
                        TCC_CTRLA_PRESCALER_DIV64;  
      break;
    }

    case 20:{
      //// Set the TCC0 PER register to generate a 0.066666 second period 
      TCC0->PER.reg = 37499;                           
      while (TCC0->SYNCBUSY.bit.PER); 
      //// Set the TCC0 CC1 register to generate high voltage within 0.0001 second             
      TCC0->CC[1].reg = 75;                          
      while (TCC0->SYNCBUSY.bit.CC1);  
      // // Reset timer on the next prescaler clock and Set prescaler to 256, 48MHz/64 = 750000
      TCC0->CTRLA.reg = TCC_CTRLA_PRESCSYNC_PRESC | 
                        TCC_CTRLA_PRESCALER_DIV64; 
      break;
    }

    default:
      break;
  }

 
  //// Enable TCC0 overflow (OVF) interrupts
  TCC0->INTENSET.reg = TCC_INTENSET_OVF;         
  // Enable the TCC0 timer
  TCC0->CTRLA.bit.ENABLE = 1;                    
  while (TCC0->SYNCBUSY.bit.ENABLE);             

  msg_info_.data = "stereo-cam is initialized";
  publisher_info_.publish(&msg_info_); 
}

void Cam::setClock(bool utc_clock, uint32_t start_time, uint32_t curr_time_base) {
  utc_clock_      = utc_clock;
  start_time_     = start_time;
  curr_time_base_ = curr_time_base;
}

void Cam::setTimeNow() {
  if (TCC0->INTFLAG.bit.OVF && TCC0->INTENSET.bit.OVF)      // Optionally check for overflow (OVF) interrupt      
  {   
    TCC0->INTFLAG.bit.OVF = 1;                                     // Clear the overflow (OVF) interrupt flag

    time_ = micros();
    msg_number_++;
    available_ = true;
  }  
}

void Cam::setNotAvailable() {
  available_ = false;
}

void Cam::exposurePrimary() {
  exposure_pri_ = micros() - time_;
}

void Cam::exposureSecondary() {
  exposure_sec_ = micros() - time_;
}

void Cam::publish() {

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

void Cam::encodeTimeROS(uint32_t curr_sec, uint32_t curr_micro) {

  msg_time_.time = ros::Time(curr_sec, curr_micro*1000);
  msg_time_.number = msg_number_;
  msg_time_.exposure_pri = exposure_pri_;
  msg_time_.exposure_sec = exposure_sec_;
}

void Cam::publishTimeROS() {

  // send time to onboard computer synchronizer_ros
  publisher_time_.publish( &msg_time_ ); 

  if(!initialized_){
    msg_info_.data = "stereo-cam is initializing";
    publisher_info_.publish(&msg_info_); 
  }
}
