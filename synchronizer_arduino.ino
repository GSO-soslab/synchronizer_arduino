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
#include "Cam.h"
// Servo
#include <Servo.h> 
//// Science system
#include "Science.h"
//// Battery
// #include "Battery.h"
//// Dvl
#include "Dvl.h"


//// Instantiate the Serial2 class to send NMEA string from Arduino to Jetson
Uart Serial2(&sercom1, SERIAL2_RX_PIN, SERIAL2_TX_PIN, SERIAL2_RX_PAD, SERIAL2_TX_PAD);

//// some global variables
volatile uint16_t offset = 0;

//// LED setting for PWM control brightness
volatile led_modes led_mode = LED_MODE_SERVO;
volatile int led_pwm = LED_PWM_MIN;

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
// callback for LED brightness value
void ledCallback( const std_msgs::String& msg);
// callback for Servo rotate position
void servoCmdCallback( const std_msgs::UInt16& msg);
// callback for science task id from onboard computer
void sciTaskCallback(const std_msgs::String &msg);

// subscribers for system level
ros::Subscriber<std_msgs::Bool> reset_sub("/rov/synchronizer/reset_system", &resetCallback);
ros::Subscriber<std_msgs::UInt32> clock_sub("/rov/synchronizer/reset_clock", &clockCallback);
// subscribers for sub_system level (science system, LED, Servo ...)
ros::Subscriber<std_msgs::String> sciTask_sub("/rov/synchronizer/science/task", &sciTaskCallback);
ros::Subscriber<std_msgs::String> led_sub("/rov/synchronizer/led/cmd", &ledCallback);
ros::Subscriber<std_msgs::UInt16> servoCmd_sub("/rov/synchronizer/servo/cmd", &servoCmdCallback);

/* ==================== Objects ==================== */

//// Instantiate Camera object, controlled by Timer TCC0
Cam cam(&nh, CAM_TOPIC, CAM_TRIGGER_PIN, CAM_RATE);
//// Instantiate Dvl object, controlled by Timer TCC1
Dvl dvl(&nh, DVL_TOPIC, DVL_TRIGGER_PIN);
//// Instantiate PPS object, controlled by Timer TCC2
Pps pps(&nh, PPS_TOPIC, PPS_TRIGGER_PIN, &Serial2);
//// Instantiate Science object
// Science sci(&nh, SCIENCE_TOPIC, &Serial1); 
//// Instantiate Battery object
// Battery battery(&nh, BATTERY_TOPIC);
//// Instantiate LED object
Servo led;
//// Instantiate Servo object
// Servo servo;

void setup() { 
/* -----  Sub-system Setting ----- */

  //// start serial2 to send NMEA to Jetson
  Serial2.begin(115200);
  Serial2.setTimeout(10);

  //// Power sensing: setup analog resoluation
  analogReadResolution(10);
  analogReference(AR_DEFAULT);

  //// Science system communication
  // Serial1.begin(9600);
  // Serial1.setTimeout(10);

  //// LED
  led.attach(LED_TRIGGER_PIN);
  led.writeMicroseconds(LED_PWM_MIN);

  //// Servo
  // servo.attach(SERVO_TRIGGER_PIN); 
  // servo.write(SERVO_FORWARD_POS);

/* ----- ROS ----- */
  //// init
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  //// sub
  nh.subscribe(reset_sub);
  nh.subscribe(clock_sub);
  nh.subscribe(sciTask_sub);
  nh.subscribe(led_sub);
  nh.subscribe(servoCmd_sub);
  //// pub
  nh.advertise(msg_pub);

  delay(1000);

  //// TODO: check every sub_module coniguration is right
  ////       but any ros message generated here will not seen in onboard computer
  ////       becasue it's too fast

/* ----- timer ----- */

  //// feed GCLK0 to TCC0 and TCC1 timers
  REG_GCLK_CLKCTRL = static_cast<uint16_t>(
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC0_TCC1);
  while (GCLK->STATUS.bit.SYNCBUSY);              
    //// feed GCLK0 to TCC2 and TC3 timers
  REG_GCLK_CLKCTRL = static_cast<uint16_t>(
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3);    
  while (GCLK->STATUS.bit.SYNCBUSY);   

  //// enable InterruptVector.
  NVIC_SetPriority(TCC0_IRQn, 0);                
  NVIC_EnableIRQ(TCC0_IRQn);
  NVIC_SetPriority(TCC1_IRQn, 0);                
  NVIC_EnableIRQ(TCC1_IRQn);
  NVIC_SetPriority(TCC2_IRQn, 0);                
  NVIC_EnableIRQ(TCC2_IRQn);

  //// Initialize all connected cameras: match image number with onboard computer sothat it can switch image time
  //// manually send init flag to make camera to initialized for test(if don't need actual image time)

  cam.initialize();
  dvl.initialize();

/* ----- Interrupt for measuring the exposure time. ----- */

  noInterrupts(); 
  attachInterrupt(digitalPinToInterrupt(CAM_PRI_EXPOSURE_PIN), exposureStatePrimary, FALLING);
  attachInterrupt(digitalPinToInterrupt(CAM_SEC_EXPOSURE_PIN), exposureStateSecond, FALLING);
  interrupts();

/* -----  Info the system to onbaord computer ----- */
  delay(5);
  str_msg.data = "LED is Servo_mode";
  msg_pub.publish(&str_msg);

  //// TODO: add more infomrations, like NMEA string baud rate.....
}

void loop() {
  //// heandle image time message publishing
  cam.publish();

  //// heandle dvl time message publishing
  dvl.publish();

  //// heandle time message publishing, take 0.03 second?
  pps.publish();

  //// Handle battery info publishing
  // battery.measurement();

  //// communication with sciecen system
  // sci.communicate();

  nh.spinOnce();
}

/* ==================== Arduino Hardware Interrupts ==================== */
void SERCOM1_Handler()    
{
  Serial2.IrqHandler();
}

void TCC0_Handler() { 
  //// Turn on LED before camera exposure
  if(led_mode == LED_MODE_FLASH)
    digitalWrite(LED_TRIGGER_PIN, HIGH);

  cam.setTimeNow();
}

void TCC1_Handler() { 
  dvl.setTimeNow();
}

void TCC2_Handler() {
  pps.setTimeNow();
  // offset = pps.getOffset();
}

void exposureStatePrimary() {
  cam.exposurePrimary();
}

void exposureStateSecond() {

  cam.exposureSecondary();

  //// Turn off LED after second camera exposure finished
  if(led_mode == LED_MODE_FLASH)
    digitalWrite(LED_TRIGGER_PIN, LOW);
}

/* ==================== ROS callbacks ==================== */

// callback for LED brightness value
void ledCallback( const std_msgs::String& msg){

  int size = strlen(msg.data);

  //// check string is consistent
  if(msg.data[0] == '#' && msg.data[size-1] == '*') {
    //// except header:'#'; mode:'0'; delimiter:','; end:'*';

    //// use the decoded data
    switch (msg.data[1]) {
      //// led mode
      case '0': {
        //// parse data
        char data[2];
        strncpy(data, &msg.data[3], 1);
        led_mode = (led_modes) atoi(data);

        if(led_mode == LED_MODE_SERVO){
          //// set to servo mode and close LED
          led.attach(LED_TRIGGER_PIN);
          led.writeMicroseconds(LED_PWM_MIN);

          str_msg.data = "#LED:servo mode";
        }
        else if(led_mode == LED_MODE_FLASH){
          //// set to flash mode and close LED
          led.detach();
          pinMode(LED_TRIGGER_PIN, OUTPUT); 
          digitalWrite(LED_TRIGGER_PIN, LOW);

          str_msg.data = "#LED:flash mode";
        }

        msg_pub.publish(&str_msg);
        break;
      }

      //// led pwm
      case '1': {
        //// parse data
        char data[5];
        strncpy(data, &msg.data[3], 4);
        led_pwm = atoi (data);

        if(led_mode == LED_MODE_SERVO){
          //// send pwm command to adjust brightness
          led.writeMicroseconds(led_pwm); 
          //// send info to onboard computer
          str_msg.data = String("#LED: PWM received " + String(led_pwm)).c_str() ;
        }
        else if (led_mode == LED_MODE_FLASH){
          //// send info to onboard computer
          str_msg.data = String("#LED: wrong mode for PWM " + String(led_pwm)).c_str() ;
        }

        msg_pub.publish(&str_msg);   
        break;
      }

      default:
        break;
    }

  }
  else {
    str_msg.data = "#LED: wrong cmd";
    msg_pub.publish(&str_msg); 
  }

}

//// callback for Servo rotation position
void servoCmdCallback( const std_msgs::UInt16& msg){

  // int data = msg.data;
  // //// Info to onboard computer 
  // str_msg.data = String("#Servo: rotate received " + String(data)).c_str() ;
  // msg_pub.publish(&str_msg); 
  
  // if(msg.data >= SERVO_FORWARD_POS)
  //   servo.write(SERVO_FORWARD_POS); //set to forward-looking limition. in case borken sonar cover
  // else if (msg.data <= SERVO_UP_POS)
  //   servo.write(SERVO_UP_POS); //set to up-looking limition.
  // else
  //   servo.write(msg.data); //set servo angle, should be from 0-180
}

// callback for setting arduino clock
// check Epoch time: https://www.epochconverter.com/
void clockCallback(const std_msgs::UInt32 &msg) { 
  if(msg.data > TIME_BASE) {

    //// start PPS generation 
    pps.begin();

    //// record arduino clock received UTC clock
    
    // uint32_t start_millis = millis();

    uint32_t start_time = micros();
    bool utc_clock = true;
    uint32_t curr_time_base = msg.data;

    //// setup cam clock
    cam.setClock(utc_clock, start_time, curr_time_base);
    //// setup dvl clock
    dvl.setClock(utc_clock, start_time, curr_time_base);
    //// setup pps clock
    pps.setClock(utc_clock, start_time, curr_time_base);
    //// setup science system clock
    // sci.setClock(utc_clock, start_time, curr_time_base);

    //// Info to onboard computer 
    str_msg.data = "#system:Arduino clock reset as UTC";
    msg_pub.publish(&str_msg);
  }
  else {
    //// Info to onboard computer 
    str_msg.data = "#system:sent clock is too old";
    msg_pub.publish(&str_msg);
  }
}

// callback for science task id from onboard computer
void sciTaskCallback(const std_msgs::String &msg) {
  // sci.publish(msg.data);
}