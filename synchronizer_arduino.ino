// Import all settings for the chosen sensor configuration.
#include "configuration.h"

// Arduino 
#include "Arduino.h"
#include <math.h>

// ROS
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

// PPS time
#include "Pps.h"
// Camera
#include "Camera.h"
#include "Timer.h"
// Servo
#include <Servo.h> 
//// Science system
#include "Science.h"
//// Battery
#include "Battery.h"

#include "helper.h"

//// Instantiate the Serial2 class
Uart Serial2(&sercom1, SERIAL2_RX_PIN, SERIAL2_TX_PIN, SERIAL2_RX_PAD, SERIAL2_TX_PAD);

// some global variables
volatile uint16_t offset = 0;

// LED setting
volatile bool led_mode = LED_TRIGGER_MODE; // true: trigger LED when camera do exposure; false: always on
volatile int led_brightness = LED_MIN_LIGHT;

/* ==================== ROS ==================== */
ros::NodeHandle nh;
// published messages
std_msgs::String str_msg;

// publishers for for system level
ros::Publisher msg_pub("/rov/synchronizer/system", &str_msg);

// callback for reset flag from onboard computer
void resetCallback(const std_msgs::Bool &msg) { NVIC_SystemReset(); }
// callback for setting arduino clock
void clockCallback(const std_msgs::UInt32 &msg);
// callback for LED flash mode
void ledModeCallback(const std_msgs::Bool & msg);
// callback for LED brightness value
void ledCmdCallback( const std_msgs::UInt16& msg);
// callback for Servo rotate position
void servoCmdCallback( const std_msgs::UInt16& msg);
// callback for science task id from onboard computer
void sciTaskCallback(const std_msgs::String &msg);

// subscribers for system level
ros::Subscriber<std_msgs::Bool> reset_sub("/rov/synchronizer/reset_system", &resetCallback);
ros::Subscriber<std_msgs::UInt32> clock_sub("/rov/synchronizer/reset_clock", &clockCallback);
// subscribers for sub_system level (science system, LED, Servo ...)
ros::Subscriber<std_msgs::String> sciTask_sub("/rov/synchronizer/science/task", &sciTaskCallback);
ros::Subscriber<std_msgs::UInt16> ledCmd_sub("/rov/synchronizer/led/cmd", &ledCmdCallback);
ros::Subscriber<std_msgs::Bool> ledMode_sub("/rov/synchronizer/led/mode", &ledModeCallback);
ros::Subscriber<std_msgs::UInt16> servoCmd_sub("/rov/synchronizer/servo/cmd", &servoCmdCallback);

/* ==================== Objects ==================== */

//// Instantiate Camera object
Timer timer_cam = Timer((Tcc *)TCC0);
Camera cam(&nh, CAM_TOPIC, CAM_RATE, timer_cam, CAM_TYPE, CAM_TRIGGER_PIN, CAM_EXPOSURE_PIN, false);
//// Instantiate PPS object
Pps pps(&nh, PPS_TOPIC, PPS_TRIGGER_PIN, &Serial2);
//// Instantiate Science object
Science sci(&nh, SCIENCE_TOPIC, &Serial1); 
//// Instantiate Battery object
Battery battery(&nh, BATTERY_TOPIC);
//// Instantiate LED object
Servo led;
//// Instantiate Servo object
Servo servo;

void setup() { 
  delay(1000);

/* ----- ROS ----- */
  //// init
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  //// sub
  nh.subscribe(reset_sub);
  nh.subscribe(clock_sub);
  nh.subscribe(sciTask_sub);
  nh.subscribe(ledCmd_sub);
  nh.subscribe(ledMode_sub);
  nh.subscribe(servoCmd_sub);
  //// pub
  nh.advertise(msg_pub);

/* -----  Camera Setting ----- */
  cam.setup();

  //// keep waitting if not configuraed right
  while (!cam.isConfigured()) {
    str_msg.data = "E:Stereo-Camera are not right configured";
    msg_pub.publish(&str_msg);
    //// keep publish this error message to onboard computer
    nh.spinOnce();
    delay(1000);
  }

  //// Initialize all connected cameras: match image number with onboard computer sothat it can switch image time
  //// manually send init flag to make camera to initialized for test(if don't need actual image time)
  while (!cam.isInitialized()) {
    str_msg.data = "I:Stereo-Camera are initializing";
    msg_pub.publish(&str_msg);

    cam.initialize();
    nh.spinOnce();
    delay(1000);
  }
  //// Info onboard computer, camera is initialized 
  str_msg.data = "I:Stereo-Camera are initialized !!";
  msg_pub.publish(&str_msg);

  //// feed GCLK0 to TCC0 and TCC1 timers
  REG_GCLK_CLKCTRL = static_cast<uint16_t>(
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC0_TCC1);
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  //// enable InterruptVector.
  NVIC_SetPriority(TCC0_IRQn, 0);                
  NVIC_EnableIRQ(TCC0_IRQn);

  cam.begin();

/* ----- Interrupt for measuring the exposure time. ----- */

  noInterrupts(); 
  attachInterrupt(digitalPinToInterrupt(cam.exposurePin()), exposureStatePrimary, FALLING);
  attachInterrupt(digitalPinToInterrupt(CAM_SEC_EXPOSURE_PIN), exposureStateSecond, FALLING);
  interrupts();

/* -----  PPS Setting ----- */

  // feed GCLK0 to TCC2 and TC3 timers
  REG_GCLK_CLKCTRL = static_cast<uint16_t>(
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3);    
  while (GCLK->STATUS.bit.SYNCBUSY);              
  // enable InterruptVector
  NVIC_SetPriority(TCC2_IRQn, 0);                
  NVIC_EnableIRQ(TCC2_IRQn);

  // start serial2 to send NMEA to Jetson
  Serial2.begin(115200);
  Serial2.setTimeout(10);

/* -----  Sub-system Setting ----- */
  //// LED
  led.attach(LED_TRIGGER_PIN);
  led.write(LED_MIN_LIGHT);

  //// Servo
  servo.attach(SERVO_TRIGGER_PIN); 
  servo.write(SERVO_FORWARD_POS);

  //// Power sensing: setup analog resoluation
  analogReadResolution(10);
  analogReference(AR_DEFAULT);

  //// Science system communication
  Serial1.begin(9600);
  Serial1.setTimeout(10);
}

void loop() {
  //// heandle image time message publishing
  cam.publish();

  //// heandle time message publishing, take 0.002 second
  pps.publish();

/* ==================== Handle battery info publishing ==================== */
  battery.measurement();

  //// receive message from sciecen system
  sci.receive();

  nh.spinOnce();
}

/* ==================== Arduino Hardware Interrupts ==================== */
void SERCOM1_Handler()    
{
  Serial2.IrqHandler();
}

void TCC2_Handler() {
  pps.setTimeNow();
  // offset = pps.getOffset();
}

void TCC0_Handler() { 
  //// flash start before cam is trigged
  // if(led_mode)
  //   led.write(led_brightness); 

  cam.triggerMeasurement();
}


void exposureStatePrimary() {
  cam.exposureEndPrimary();
}

void exposureStateSecond() {
  //// LED only close after the secondary camera finish exposure
  //// since secondary camera exposure will late 80 us after primary camera
  cam.exposureEndSecond();

  //// flash end after cam is trigged
  // if(led_mode)
  //   led.write(LED_MIN_LIGHT); 
}

/* ==================== ROS callbacks ==================== */

// callback for LED flash mode:
void ledModeCallback(const std_msgs::Bool & msg) { 
  led_mode = msg.data; 

  //// Info to onboard computer 
  if(msg.data)
    str_msg.data = "#LED:flash with cam";
  else
    str_msg.data = "#LED:flash all the time";

  msg_pub.publish(&str_msg); 
}

// callback for LED brightness value
void ledCmdCallback( const std_msgs::UInt16& msg){

  led_brightness = msg.data;

  //// Info to onboard computer 
  str_msg.data = String("#LED: brightness received " + String(led_brightness)).c_str() ;
  msg_pub.publish(&str_msg);   

  if(!led_mode)
    led.write(led_brightness); // directly write to LED 
}

//// callback for Servo rotation position
void servoCmdCallback( const std_msgs::UInt16& msg){

  int data = msg.data;
  //// Info to onboard computer 
  str_msg.data = String("#Servo: rotate received " + String(data)).c_str() ;
  msg_pub.publish(&str_msg); 
  
  if(msg.data >= SERVO_FORWARD_POS)
    servo.write(SERVO_FORWARD_POS); //set to forward-looking limition. in case borken sonar cover
  else if (msg.data <= SERVO_UP_POS)
    servo.write(SERVO_UP_POS); //set to up-looking limition.
  else
    servo.write(msg.data); //set servo angle, should be from 0-180
}

// callback for setting arduino clock
// check Epoch time: https://www.epochconverter.com/
void clockCallback(const std_msgs::UInt32 &msg) { 
  if(msg.data > TIME_BASE) {
    //// Info to onboard computer 
    str_msg.data = "#system:Arduino clock reset as UTC";
    msg_pub.publish(&str_msg);

    //// start PPS generation 
    pps.begin();

    //// record arduino clock received UTC clock
    uint32_t start_time = micros();
    bool utc_clock = true;
    uint32_t curr_time_base = msg.data;

    //// setup science system clock
    sci.setClock(utc_clock, start_time, curr_time_base);
    //// setup pps clock
    pps.setClock(utc_clock, start_time, curr_time_base);
    //// setup cam clock
    cam.setClock(utc_clock, start_time, curr_time_base);
  }
  else {
    //// Info to onboard computer 
    str_msg.data = "#system:sent clock is too old";
    msg_pub.publish(&str_msg);
  }
}

// callback for science task id from onboard computer
void sciTaskCallback(const std_msgs::String &msg) {

  sci.publish(msg.data);

}