////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Camera.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides functions regarding cameras in the versavis
//  package. Refer to the parent package versavis for license
//  information.
//
////////////////////////////////////////////////////////////////////////////////

#include "Camera.h"
#include "helper.h"
#include "configuration.h"

Camera::Camera(ros::NodeHandle *nh, const String &topic, const int rate_hz,
               Timer &timer, const trigger_type &type,
               const uint8_t trigger_pin /*= 0 */,
               const uint8_t exposure_pin /*= 0 */,
               const bool exposure_compensation /*= true*/)
    : Sensor(nh, topic, rate_hz, timer, image_time_msg_, test_msg_, info_msg_, type),
      trigger_pin_(trigger_pin), exposure_pin_(exposure_pin),
      exposure_compensation_(exposure_compensation), configured_(true),
      compensating_(false), exposing_(false), image_number_(0),
      init_subscriber_((topic + "init").c_str(), &Camera::initCallback, this),
      initialized_(false), initialized_cam_(0),
      exposure_pri_(0), exposure_sec_(0)
{
  Sensor::newMeasurementIsNotAvailable();
}

void Camera::setup() {

  /******************** ROS sub and pub ********************/
  setupPublisher();

  setupInitSubscriber();

  /******************** configuration check ********************/
  //// trigger pin
  if (trigger_pin_ == 0) {
    info_msg_.data = "E:Trigger pin is set to 0.";
    publisher_information_.publish(&info_msg_);
    
    configured_ = false;
    return;
  }
  //// exposure pin
  if (exposure_pin_ == 0) {
    info_msg_.data = "E:Exposure pin is set to 0.";
    publisher_information_.publish(&info_msg_);

    configured_ = false;
    return;
  }
  //// rostopic 
  if (topic_.length() == 0) {
    info_msg_.data = "E:no rostopic for cameras";
    publisher_information_.publish(&info_msg_);

    configured_ = false;
    return;
  }

  /******************** pin mode assign ********************/
  pinMode(trigger_pin_, OUTPUT);
  digitalWrite(trigger_pin_, LOW);
  //// used for exposure compensate, not used right now
  pinMode(exposure_pin_, INPUT);
}

void Camera::initialize() {
  // trigger camera to get one image
  Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US * 10, type_);
  Sensor::setTimestampNow(false, 0, 0);

  ++image_number_;
  image_time_msg_.number = image_number_;
  Sensor::newMeasurementIsAvailable();
  publish();
}

void Camera::begin() {

  // Maximal exposure time to still be able to keep up with the frequency
  // considering a security factor of 0.99, in us.
  max_exposure_time_us_ = 0.99 * 1e6 / rate_hz_;

  // Setup timer to periodically trigger the camera.
  Sensor::setupTimer();
}

void Camera::setupPublisher() {
  //// send all the camera system information to onboard computer
  publisher_information_ = ros::Publisher("/rov/synchronizer/cam/system_info", &info_msg_);

  //// send image time to onboard computer
  publisher_ = ros::Publisher("/rov/synchronizer/cam/image_time", &image_time_msg_);

  //// send exposure time to onboard computer
  publisher_test = ros::Publisher("/rov/synchronizer/cam/exposure_time", &test_msg_);

  nh_->advertise(publisher_);
  nh_->advertise(publisher_test);
  nh_->advertise(publisher_information_);
}

void Camera::setupInitSubscriber() {
  init_subscriber_ = ros::Subscriber<std_msgs::Bool, Camera>(
                     "/rov/synchronizer/cam/init", &Camera::initCallback, this);
  nh_->subscribe(init_subscriber_);
}

void Camera::initCallback(const std_msgs::Bool &msg) {
  // two cameras (stereo) found synchronization offsets
  if (msg.data)
    initialized_cam_++;

  if (initialized_cam_==2)
    initialized_ = true;
}

void Camera::triggerMeasurement(bool utc_clock, uint32_t curr_time_base, uint32_t start_time) {
  // Check whether an overflow caused the interrupt.
  if (!timer_.checkOverflow()) {
    info_msg_.data = "W:Timer interrupt but not overflown.";
    publisher_information_.publish(&info_msg_);
    return;
  }
  if (!initialized_) {
    return;
  }

  if (exposure_compensation_ && compensating_) {
    // Exposure-time compensating mode (Nikolic 2014). During exposure, the
    // timer will interrupt in the middle of the exposure time. At the end of
    // the exposure, the external interrupt will trigger exposureEnd() and
    // reset the timer to trigger the camera at the appropriate time.
    if (!exposing_) {
      // The camera is currently not exposing meaning that the interrupt
      // triggers at the beginning of the next image.
      exposing_ = true;

#ifdef ILLUMINATION_MODULE
      // If the illumination module is active, the LEDs should turn on just
      // before the camera is exposing.
      digitalWrite(ILLUMINATION_PIN, HIGH);
      // Here, a warm-up delay for the LEDs can be added (needs checking).
      // delayMicroseconds(10);
#endif
      // Trigger the actual pulse.
      Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US, type_);

      // Save the current time to estimate the exposure time in the pin
      // interrupt.
      exposure_start_us_ = micros();

      // Increament the image number as the camera is triggered now.
      ++image_number_;

      // Set the timer to the mid exposure point, e.g. half the exposure time.
      timer_.setCompare(exposure_delay_ticks_ > 0 ? exposure_delay_ticks_ - 1
                                                  : compare_);
    } else {
      // The camera is currently exposing. In this case, the interrupt is
      // triggered in the middle of the exposure time, where the timestamp
      // should be taken.
      Sensor::setTimestampNow(utc_clock, curr_time_base, start_time);
      Sensor::newMeasurementIsAvailable();
#ifdef ADD_TRIGGERS
      trigger(ADDITIONAL_TEST_PIN, TRIGGER_PULSE_US,
              Sensor::trigger_type::NON_INVERTED);
#endif

      // Even though we are still in the compensating mode, deactivating here
      // ensures that we detect if a exposure signal is dropped and we switch
      // to non-compensating mode.
      compensating_ = false;

      // Set the timer to the standard period as we dont know the current
      // exposure time yet.
      timer_.setCompare(compare_);
    }
  } else {
    // "Standard" mode where the camera is triggered purely periodic.
    exposing_ = true;

    //// TODO: open led here

    // Trigger the actual pulse.
    Sensor::trigger(trigger_pin_, TRIGGER_PULSE_US, type_);

    Sensor::setTimestampNow(utc_clock, curr_time_base, start_time);
    Sensor::newMeasurementIsAvailable();

    // Save the current time to estimate the exposure time in the pin
    // interrupt.
    exposure_start_us_ = micros();

    // Increament the image number as the camera is triggered now.
    image_number_++;

    // Set the timer to make sure that the camera is triggered in a periodic
    // mode.
    timer_.setCompare(compare_);
  }
  // Reset the timer.
  timer_.resetOverflow();
}

void Camera::exposureEndPrimary() {

 // exposure time ~ 10043 us, when set to static 10000 us exposure, 
 // should include 27 us trigger delay
  exposure_pri_ = micros() - exposure_start_us_;

  // Sensor::newMeasurementIsAvailable();

  if (exposure_compensation_) {
    unsigned long last_exposure_time_us = micros() - exposure_start_us_;
    DEBUG_PRINT((topic_ + " (Camera.cpp): exposure time [us] ").c_str());
    DEBUG_PRINTDECLN(last_exposure_time_us);
    calculateDelayTicksAndCompensate(last_exposure_time_us);
    exposing_ = false;
  }
}

void Camera::exposureEndSecond() {

 // exposure time ~ 10132, when primary camera set to static 10000 us exposure
  exposure_sec_ = micros() - exposure_start_us_;

  // Sensor::newMeasurementIsAvailable();

}

void Camera::publish() {
  //// make sure parimary cam is trigged, 
  //// make sure parimary cam is exposured
  //// make sure secondary cam is exposured
  if (Sensor::isNewMeasurementAvailable()) {
    image_time_msg_.time = Sensor::getTimestamp();
    image_time_msg_.number = image_number_;

    image_time_msg_.exposure_pri = exposure_pri_;
    image_time_msg_.exposure_sec = exposure_sec_;

    publisher_.publish(&image_time_msg_);

    Sensor::newMeasurementIsNotAvailable();
  }


}

void Camera::calculateDelayTicksAndCompensate(
    const unsigned long &last_exposure_time_us) {
  // The goal here is to shift the time of the next camera trigger half the
  // exposure time before the mid-exposure time.

  // The next frame should be triggered by this time before the mid-exposure
  // time (in CPU ticks).
  if (last_exposure_time_us == 0 ||
      last_exposure_time_us >= max_exposure_time_us_) {
    // In this case, something with the measurement went wrong or the camera
    // is dropping triggers due to a too high exposure time (constrain the
    // maximal exposure time of your camera to be within the period time of
    // your triggering). Switch to non-compensating mode.
    exposure_delay_ticks_ = 0;
    compensating_ = false;
  } else {
    exposure_delay_ticks_ = static_cast<double>(last_exposure_time_us) / 2.0 /
                            1000000.0 * cpu_freq_prescaler_;
    compensating_ = true;
  }

  // Reset the compare register of the timer.
  timer_.setCompare(compare_ - exposure_delay_ticks_);
}
