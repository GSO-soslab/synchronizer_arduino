#ifndef CONFIGURATION_H_ 
#define CONFIGURATION_H_

#include "Arduino.h"

/* ----- General configuration -----*/
// Activate USB serial interface for ARM processor types.
// put this first 
#define USE_USBCON

// Specify the CPU frequency of the controller.
#define CPU_FREQ_HZ 48e6

// Specify the trigger pulse width;
#define TRIGGER_PULSE_US 10 // 10 microsecond = 0.000010s; 

/* ----- serial2  configuration ---- */

//  Serial2 (on Sercom1) using digital pins 12 (Rx) and 10 (Tx).
// [check here](https://forum.arduino.cc/t/arduino-zero-softwareserial-library/328806/6)
#define SERIAL2_RX_PIN       (34ul)               // Pin description number for PIO_SERCOM on D12
#define SERIAL2_TX_PIN       (36ul)               // Pin description number for PIO_SERCOM on D10
#define SERIAL2_RX_PAD       (SERCOM_RX_PAD_3)    // SERCOM pad 3
#define SERIAL2_TX_PAD       (UART_TX_PAD_2)      // SERCOM pad 2

/* ----- sensors configuration ---- */

// arduino system time beginning
#define TIME_BASE 1621258639

//// All times:  TCC0, TCC1, TCC2, TC3, TC4, TC5

//// Camera: TCC0
#define CAM_TOPIC "/rov/synchronizer/cam/"
#define CAM_RATE 20 //5,10,15,20
#define CAM_TYPE trigger_type::NON_INVERTED
#define CAM_TRIGGER_PIN 3
#define CAM_PRI_EXPOSURE_PIN 5
#define CAM_SEC_EXPOSURE_PIN 8

//// DVL tigger: TCC1
#define DVL_TOPIC "/rov/synchronizer/dvl/"
#define DVL_RATE 8
#define DVL_TYPE trigger_type::NON_INVERTED
#define DVL_TRIGGER_PIN 9

//// Pps: TCC2
#define PPS_TOPIC "/rov/synchronizer/pps/"
#define PPS_TRIGGER_PIN 13
#define PPS_TIME_DELAY 10 // send pps time after this delay

//// Servo library: TC4 
//// Servo
#define SERVO_TOPIC "/rov/synchronizer/servo/"
#define SERVO_TRIGGER_PIN 14
#define SERVO_UP_POS 30
#define SERVO_FORWARD_POS 64

//// LED
enum led_modes{
  LED_MODE_SERVO = 0,
  LED_MODE_FLASH = 1
};

#define LED_TOPIC "/rov/synchronizer/led/"
#define LED_TRIGGER_PIN 16
#define LED_PWM_MIN 1100          
#define LED_PWM_max 1900          

//// Battery
#define BATTERY_TOPIC "/rov/synchronizer/battery/"
#define BATTERY_MEASURE_TIME 1000000 // 1 second
#define BATTERY_PUB_SIZE 5
#define BATTERY_WARNING 12.0
#define BATTERY_CURRENT_PIN A4
#define BATTERY_VOLTAGE_PIN A5

//// Science system
#define SCIENCE_TOPIC "/rov/synchronizer/science/"

/* ---- Test mode ----- */
// send test information through ROS rostopic 
// we not avaiable arduino serial print when everthing in the ROV
#define TEST



// extern volatile uint16_t offset;
// extern volatile uint32_t curr_time_base;
// extern volatile bool reset;

#endif // CONFIGURATION_H_
