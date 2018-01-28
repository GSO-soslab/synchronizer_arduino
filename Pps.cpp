#include "Pps.h"



Pps::Pps(ros::NodeHandle *nh, const String &topic, const uint8_t trigger_pin, HardwareSerial* serial) 
  : nh_(nh),  trigger_pin_(trigger_pin), available_(false), micro_offset_(0),
    topic_time_(topic+"time"), publisher_time_(topic_time_.c_str(), &pps_time_msg_),
    topic_info_(topic+"info"), publisher_info_(topic_info_.c_str(), &pps_info_msg_)
{
  publisher_time_ = ros::Publisher("/rov/synchronizer/pps/time", &pps_time_msg_);
  publisher_info_ = ros::Publisher("/rov/synchronizer/pps/info", &pps_info_msg_);

  nh_->advertise(publisher_time_);
  nh_->advertise(publisher_info_);

  serial2_ = serial;
}

void Pps::begin() {
  /* ----- Serial setup -----*/

/** Set timer TCC2 generate a 100ms pulse every second on D13 (PA17) **/
  // Enable D13's peripheral multiplexer
  PORT->Group[g_APinDescription[trigger_pin_].ulPort].PINCFG[g_APinDescription[trigger_pin_].ulPin].bit.PMUXEN = 1;
  // Set D13 multiplexer switch to position
  PORT->Group[g_APinDescription[trigger_pin_].ulPort].PMUX[g_APinDescription[trigger_pin_].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;
  
  TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;        // Configure the TCC2 timer for normal PWM mode
  while (TCC2->SYNCBUSY.bit.WAVE);               // Wait for synchronization
  
  TCC2->PER.reg = 46874;                         // Set the TCC2 PER register to generate a 1 second period
  while (TCC2->SYNCBUSY.bit.PER);                // Wait for synchronization
  TCC2->CC[1].reg = 4688;                        // Set the TCC2 CC1 register to generate a duty cycle of 10% (100ms)
  while (TCC2->SYNCBUSY.bit.CC1);                // Wait for synchronization

  // NVIC_SetPriority(TCC2_IRQn, 0);                // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC2 to 0 (highest)
  // NVIC_EnableIRQ(TCC2_IRQn);                     // Connect TCC2 to Nested Vector Interrupt Controller (NVIC)

  TCC2->INTENSET.reg = TCC_INTENSET_OVF;         // Enable TCC2 overflow (OVF) interrupts
 
  TCC2->CTRLA.reg = TCC_CTRLA_PRESCSYNC_PRESC |  // Reset timer on the next prescaler clock
                    TCC_CTRLA_PRESCALER_DIV1024; // Set prescaler to 1024, 48MHz/1024 = 46875kHz                       
  
  TCC2->CTRLA.bit.ENABLE = 1;                    // Enable the TCC2 timer
  while (TCC2->SYNCBUSY.bit.ENABLE);             // Wait for synchronization  
}

void Pps::setTimeNow() {
  if (TCC2->INTFLAG.bit.OVF && TCC2->INTENSET.bit.OVF)      // Optionally check for overflow (OVF) interrupt      
  {   
    TCC2->INTFLAG.bit.OVF = 1;                              // Clear the overflow (OVF) interrupt flag

    time_ = micros();

    available_ = true;
  }  
}

void Pps::setNotAvailable() {
  available_ = false;
}

void Pps::publish(bool utc_clock, uint32_t curr_time_base, uint32_t start_time) {

  if(isAvailable()) {
    uint32_t t1 = micros();

    //// get time duration after UTC clock is set 
    uint32_t time_aft_utc, latest_sec;
    time_aft_utc = time_ - start_time;
    //// get second part
    if(utc_clock) 
      latest_sec = curr_time_base + time_aft_utc / 1000000;
    else
      latest_sec = TIME_BASE + time_aft_utc / 1000000;
    //// get microssecond part
    //micro_offset_ = time_aft_utc % 1000000;   

    //// encode time format
    encodeTimeROS(latest_sec);
    encodeTimeGPS(latest_sec);

    //// publish time 
    publishTimeROS();
    publishTimeGPS();

    //// set not avaiable
    setNotAvailable();

    uint32_t dt = micros() - t1;
    String str = String(dt);
    pps_info_msg_.data = str.c_str();
    publisher_info_.publish(&pps_info_msg_);
  }

}

void Pps::encodeTimeROS(uint32_t curr_time) {

  pps_time_msg_.header.stamp = ros::Time(curr_time, 0);
  pps_time_msg_.header.frame_id = "arduino";
  pps_time_msg_.time_ref = pps_time_msg_.header.stamp;
  pps_time_msg_.source   = pps_time_msg_.header.frame_id;
}

// NMEA example: http://aprs.gids.nl/nmea/
// NMEA Checksum Calculator: https://nmeachecksum.eqth.net/
// Epoch time: https://www.epochconverter.com/
// real data:
  // $GPGGA,205331.000,4129.4837,N,07125.3140,W,2,07,1.04,28.2,M,-34.4,M,0000,0000*6E
  // $GPGSA,A,3,31,22,32,25,10,12,29,,,,,,1.32,1.04,0.81*03
  // $GPGSV,2,1,08,31,75,308,43,32,59,105,44,22,38,293,37,25,37,053,26*71
  // $GPGSV,2,2,08,29,12,098,29,10,12,170,32,12,09,037,36,49,01,099,34*76
  // $GPRMC,205331.000,A,4129.4837,N,07125.3140,W,0.01,260.08,011121,,,D*71
  // $GPZDA,205331.000,01,11,2021,,*50
void Pps::encodeTimeGPS(uint32_t curr_time) {

/****** convert Epoch to UTC ******/
  time_t rawtime = curr_time;
  struct tm ts;
  char utc_time[7];
  char utc_date[7];
  char utc_DATE[9];
  char dd[3];
  char mm[3];
  char YY[5];
  // get time
  ts = *localtime(&rawtime);
  strftime(utc_time, sizeof(utc_time), "%H%M%S", &ts);
  strftime(utc_date, sizeof(utc_date), "%d%m%y", &ts);
  // for ZDA
  strftime(utc_DATE, sizeof(utc_DATE), "%d%m%Y", &ts);
  strncpy(dd, utc_DATE + 0, 2);
  strncpy(mm, utc_DATE + 2, 2);
  strncpy(YY, utc_DATE + 4, 4);
  dd[2]='\0';
  mm[2]='\0';
  YY[4]='\0';

  char end[5];
  int size, checksum;

/****** GPGGA ******/
  // $GPGGA,092751.00,41.49,N,71.25,W,1,08,1.03,61.7,M,55.3,M,,*46<CR><LF>
  //          1         2   3   4   5 6 7   8     9 10  11 12131415 16  17 
  //          |         |   |   |   | | |   |     |  |   |  |||  |  |   |
  // $GPGGA,hhmmss.ss,xx.xx,a,xx.xx,a,x,xx,x.xx,xx.x,M,xx.x,M,,*46<CR><LF>
  // 1) UTC time: 09:27:51
  // 2) Latitude: 41.49
  // 3) North/South: N
  // 4) Longitude: 71.25
  // 5) East/West: W
  // 6) GPS Quality: 0=no fix, 1=GPS fix, 2=Dif. GPS fix
  // 7) Number of satellites in use
  // 8) Horizontal Dilution of Precision (HDOP): Relative accuracy of horizontal position
  // 9) Antenna altitude above mean-sea-level: 
  // 10) M: units of antenna altitude, meters
  // 11) Height of geoid above WGS84 ellipsoid
  // 12) M: units of geoidal separation, meters
  // 13) Blank: Time since last DGPS update
  // 14) Blank: DGPS reference station id
  // 15) Checksum between $ and *: 46
  // 16) <CR>: Carriage return
  // 17) <LF>: Line feed, end delimiter 2
  checksum = 0;
  size = sprintf(gpgga, "$GPGGA,%s.00,4129.4837,N,07125.3140,W,2,07,1.04,28.2,M,-34.4,M,,", utc_time);
                  
  for(int i=1;i<size;i++) {
      checksum^=gpgga[i];
  }
  sprintf(end,"*%02X%c%c",checksum,13,10);
  
  strcat(gpgga,end);

/****** GPGSA  ******/
  // $GPGSA,A,3,10,07,05,02,29,04,08,13,,,,,1.72,1.03,1.38*0A<CR><LF>
  //        1 2  3                       14 15   16   17   18 19 20
  //        | |  |                        |  |    |    |    |  |  |  
  // $GPGSA,A,x,xx,xx,xx,xx,xx,xx,xx,xx,,,,,x.xx,x.xx,x.xx*0A<CR><LF>
  // 1) Mode: M (Manual, forced to operate in 2D or 3D); A (Automatic, 3D/2D)
  // 2) Mode: 1 (Fix not available); 2 (2D); 3 (3D)
  // 3-14) IDs: SVs used in position fix (null for unused fields) 
  // 15) PDOP
  // 16) HDOP
  // 17) VHOP
  // 18) Checksum between $ and *: 0A
  // 19) <CR>: Carriage return
  // 20) <LF>: Line feed, end delimiter
  checksum = 0;
  size = sprintf(gpgsa, "$GPGSA,A,3,31,22,32,25,10,12,29,,,,,,1.32,1.04,0.81");
  for(int i=1;i<size;i++) 
      checksum^=gpgsa[i];
  sprintf(end,"*%02X%c%c",checksum,13,10);
  strcat(gpgsa,end);

/***** GPGSV *****/
  // $GPGSV,3,1,11,10,63,137,17,07,61,098,15,05,59,290,20,08,54,157,30*70
  //        1 2 3  4  5   6  7  8  9  10 11  12 13 14  15 16  17 18  19
  //        | | |  |  |   |  |  |  |   |  |  |  |   |   |  |  |  |   |
  // $GPGSV,3,1,11,10,63,137,17,07,61,098,15,05,59,290,20,08,54,157,30*70
  // 1) Totall number of messages
  // 2) Messager number
  // 3) Total number of SVs in view
  // 4) SV PRN number
  // 5) Elevation in degrees, 90 maximum
  // 6) Azimuth, degrees from true north, 000 to 359
  // 7) SNR, 00-99 dB (null when not tracking)
  // 8-11) next SV, same as 4-7)
  // 12-15) next SV, same as 4-7)
  // 16-19) next SV, same as 4-7)

  // 1st
  checksum = 0;
  size = sprintf(gpgsv_1, "$GPGSV,2,1,08,31,75,308,43,32,59,105,44,22,38,293,37,25,37,053,26");
  for(int i=1;i<size;i++) 
      checksum^=gpgsv_1[i];
  sprintf(end,"*%02X%c%c",checksum,13,10);
  strcat(gpgsv_1,end);
  // 2nd
  checksum = 0;
  size = sprintf(gpgsv_2, "$GPGSV,2,2,08,29,12,098,29,10,12,170,32,12,09,037,36,49,01,099,34");
  for(int i=1;i<size;i++) 
      checksum^=gpgsv_2[i];
  sprintf(end,"*%02X%c%c",checksum,13,10);
  strcat(gpgsv_2,end);

/****** GPRMC ******/
  // $GPRMC,092751.00,A,41.49,N,71.25,W,1.94,66.66,280511,004.2,W,D*5C<CR><LF>
  //           1      2   3   4   5   6  7     8      9     10 1112 1314  15
  //           |      |   |   |   |   |  |     |      |      |  | | |  |  |     
  // $GPRMC,hhmmss.ss,A,xx.xx,N,xx.xx,W,x.xx,xx.xx,ddmmyy,xxx.x,W,D*HH<CR><LF>
  // 1) UTC time: 09:27:51
  // 2) Validity: A-ok, V-invalid 
  // 3) Latitude: 41.49
  // 4) North/South: N
  // 5) Longitude: 71.25
  // 6) East/West: W
  // 7) Speed in knots: 1.94 ~ 1m/s
  // 8) True north: 66.66 (fake)
  // 9) UTC Date: 28 May 2011
  // 10) Magnetic Declination (Variation): 004.2(fake)
  // 11) East/West: W
  // 12) Positioning system mode indicator: A (Autonomous), D (Differential), E (Estimated (dead reckoning) mode), M (Manual input), N (Data not valid)
  // 13) Checksum between $ and *: 5C
  // 14) <CR>: Carriage return
  // 15) <LF>: Line feed, end delimiter
  checksum = 0;
  size = sprintf(gprmc, "$GPRMC,%s.000,A,4129.4837,N,07125.3140,W,0.01,260.08,%s,,,D", utc_time, utc_date);

  for(int i=1;i<size;i++) {
      checksum^=gprmc[i];
  }
  sprintf(end,"*%02X%c%c",checksum,13,10);
  
  strcat(gprmc,end);

/****** GPZDA ******/
  // $GPZDA,205331.000,01,11,2021,,*50<CR><LF>
  //              1    2  3   4   5  6  7  8   9
  //              |    |  |   |   |  |  |  |   |
  // $GPZDA,hhmmss.sss,dd,mm,yyyy,xx,xx*50<CR><LF>
  // 1) UTC time: 20:53:31
  // 2) Day: 01
  // 3) Month: 11(Nov.)
  // 4) Year: 2021
  // 5) Local zone description, 00 to +/- 13 hours
  // 6) Local zone minutes description (same sign as hours)
  // 7) Checksum between $ and *: 5C
  // 8) <CR>: Carriage return
  // 9) <LF>: Line feed, end delimiter 
  checksum = 0;
  size = sprintf(gpzda, "$GPZDA,%s.000,%s,%s,%s,,", utc_time, dd, mm, YY);

  for(int i=1;i<size;i++) {
      checksum^=gpzda[i];
  }
  sprintf(end,"*%02X%c%c",checksum,13,10);
  
  strcat(gpzda,end);  
}

//// send time to AHRS by ROS sensor_msgs/TimeReference msg
void Pps::publishTimeROS() {
    // send pps time delay some time(30ms) after PPS is sent
    // delay(PPS_TIME_DELAY); 

    // send time to AHRS by ROS Message
    publisher_time_.publish( &pps_time_msg_ ); 
}

//// send time to Jetson by NMEA String
void Pps::publishTimeGPS() {

  serial2_->write(gpgga,  sizeof(gpgga));
  serial2_->write(gpgsa,  sizeof(gpgsa));
  serial2_->write(gpgsv_1, sizeof(gpgsv_1));
  serial2_->write(gpgsv_2, sizeof(gpgsv_2));
  serial2_->write(gprmc,  sizeof(gprmc));
  serial2_->write(gpzda,  sizeof(gpzda));

}
