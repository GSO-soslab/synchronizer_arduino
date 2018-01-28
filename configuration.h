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
#define TRIGGER_PULSE_US 10 // 10 microsecond = 0.000010s;  0.100000s for PPS, x 10000 later

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

//// PPS
#define PPS_TOPIC "/rov/synchronizer/pps/"
#define PPS_TRIGGER_PIN 13
#define PPS_TIME_DELAY 10 // send pps time after this delay

//// Cameras
#define CAM_TOPIC "/rov/synchronizer/cam/"
#define CAM_RATE 20 
#define CAM_TYPE trigger_type::NON_INVERTED
#define CAM_TRIGGER_PIN 14
#define CAM_EXPOSURE_PIN 5
#define CAM_SEC_EXPOSURE_PIN 8

//// Servo
#define SERVO_TOPIC "/rov/utilities/servo"
#define SERVO_TRIGGER_PIN 3
#define SERVO_UP_POS 30
#define SERVO_FORWARD_POS 64

//// LED
#define LED_TOPIC "/rov/utilities/led"
#define LED_TRIGGER_PIN 4
#define LED_MIN_LIGHT 40        // this value close LED
#define LED_TRIGGER_MODE false  

//// Battery
#define BATTERY_MEASURE_TIME 1000000 // 1 second
#define BATTERY_PUB_SIZE 5
#define BATTERY_WARNING 12.0
#define BATTERY_CURRENT_PIN A4
#define BATTERY_VOLTAGE_PIN A5

/* ---- Test mode ----- */
// send test information through ROS rostopic 
// we not avaiable arduino serial print when everthing in the ROV
#define TEST



// extern volatile uint16_t offset;
// extern volatile uint32_t curr_time_base;
// extern volatile bool reset;

#endif // CONFIGURATION_H_
