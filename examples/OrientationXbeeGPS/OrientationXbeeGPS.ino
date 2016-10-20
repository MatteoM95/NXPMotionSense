// This code uses NXP's advanced sensor fusion algorithm to determine the orientation of the sensors
// and transmit them at a rate of about 100 hertz. 
// The MPL3115 pressure sensor is used to calculate altitude. A GP-735 GPS is parsed using TinyGPS.
// The values are transmitted using Digi's XBee Pro S2C radio modules. These are cabable of 2 miles 
// line of sight. The Teensy Prop Shield is used as a 9DOF sensor. It contains the MPL3115 pressure
// sensor, and also has an 8M flash memory chip. About 100 bytes are being used by NXP's software to
// store calibration data. The rest could be used to store altitude data, etc. 
// 
// The data is gathered approximately every 10 ms, with the exception of the GPS data at 100ms. This
// data is transmited every 10ms (100 times a second) to the ground station.

// Full orientation sensing using NXP's advanced sensor fusion algorithm.
// You *must* perform a magnetic calibration before this code will work

//  Location of software: https://github.com/radiohound/NXPMotionSense
//  More info about project: https://hackaday.io/project/15425-rocket-real-time-transponder-and-gui
//


#include <NXPMotionSense.h>
#include <SparkFunMPL3115A2.h>
#include <Wire.h>
#include <EEPROM.h> // Used by NXPMotionSense for reading sensor calibration data. See calibration info
                    // at http://www.pjrc.com/store/prop_shield.html
#include "kalman.h" // Simplified Kalman filter for altitude accuracy and smoothing
#include <TinyGPS.h>

#define USBSerial Serial
#define XBeeSerial Serial1
#define GPSSerial Serial2

TinyGPS gps;// ----------------- From TinyGPS example https://www.pjrc.com/teensy/td_libs_TinyGPS.html
long lat, lon;
void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);  // TinyGPS settings

//Create an instance of the object
MPL3115A2 myPressure;
NXPMotionSense imu;
NXPSensorFusion filter;
KalmanFilter kf(0, 0.008, 10); //was (0, .01, 1.0) 

void setup() {
  USBSerial.begin(115200); // the usb serial ... 
  //Begin HW serial
  XBeeSerial.begin(230400); //set to highest baud rate for xbee pro module
  //XBees need to be programmed with the above baud setting. XBee Pro S1 max 115200
  //XBee pro SC2 max at 230400 baud. Need to make sure firmware in xbee is set to 802.15.4 TH PRO setting
  imu.begin();
  filter.begin(50); //this is for NXP Sensor Fusion filter
  
  myPressure.begin(); // Get mpl3115 sensor online
  //Configure the sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters (will measure in feet later)
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(2); // Set Oversample. I have this set to 2 so it doesnt get bogged down with this
  // sampling intervals of 1=6 ms , 2=10, 4=18, 8=34, 16=66, 32=130, 64=258, and 128=512ms 
  // oversampling set to 128 takes half a second to perform. Even when set to 8 there is some lag for the orientation. 
  // Seems to be ok when set to 2. There is pretty good filtering with the simplified Kalman filter
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

  //SETUP GPS
    // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);
  delay(3000);
  
  //ublox code to change baud to 115200 
  byte message[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00};
  GPSSerial.write(message, sizeof(message));
  byte message1[] = {0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2};
  GPSSerial.write(message1, sizeof(message1));
  byte message2[] = {0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00};
  GPSSerial.write(message2, sizeof(message2));
  byte message3[] = {0x00, 0x00, 0xC0, 0x7E};
  GPSSerial.write(message3, sizeof(message3));
  delay(150);
  byte message4[] = {0xB5, 0x62, 0x06, 0x86, 0x00, 0x00, 0x8C, 0xAA};
  GPSSerial.write(message4, sizeof(message4)); 
  
  GPSSerial.begin(115200);
  
  byte message5[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01};  //removing gpgll from nmea output
  GPSSerial.write(message5, sizeof(message5)); 
  byte message6[] = {0x00, 0xFB, 0x11};  
  GPSSerial.write(message6, sizeof(message6)); 

  byte message7[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02};  //removing gpgSA from nmea output
  GPSSerial.write(message7, sizeof(message7)); 
  byte message8[] = {0x00, 0xFC, 0x13};  
  GPSSerial.write(message8, sizeof(message8)); 

  byte message9[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03};  //removing gpgSV from nmea output
  GPSSerial.write(message9, sizeof(message9)); 
  byte message10[] = {0x00, 0xFD, 0x15};  
  GPSSerial.write(message10, sizeof(message10));   

  delay(100);
  byte message11[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00};  //CHANGE RATE TO 10 HZ nmea output
  GPSSerial.write(message11, sizeof(message11)); 
  byte message12[] = {0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};  
  GPSSerial.write(message12, sizeof(message12)); 

  byte message13[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04};  //removing gPRMC from nmea output
  GPSSerial.write(message13, sizeof(message13)); 
  byte message14[] = {0x00, 0xFE, 0x17};  
  GPSSerial.write(message14, sizeof(message14)); 

  byte message15[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05};  //removing gBVTG from nmea output
  GPSSerial.write(message15, sizeof(message15)); 
  byte message16[] = {0x00, 0xFF, 0x19};  
  GPSSerial.write(message16, sizeof(message16));   

  //END GPS SETUP FOR single sentence GPGGA, rate 10 hertz, and baud set to 115200  -Specifically set GP-735 gps (ublox 7 series firmware)
}

void loop() {
  //elapsedMillis timeElapsed;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;
  
   
  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
    // Update the SensorFusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    float alt = (float) myPressure.readAltitudeFt(); // gets float of altitude with two decimal places for kalman filtering
    float alt_kalman = kf.step(alt); //filter noise from altimeter sensor, and get readings to settle

    // print the orientation, GPS, and altitude parameters to the xbee serial port
    // kept decimals in for these variables because I liked the smoother movement in the processing display
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    XBeeSerial.print("O: ");
    XBeeSerial.print(heading);
    XBeeSerial.print(" ");
    XBeeSerial.print(pitch); //pitch
    XBeeSerial.print(" ");
    XBeeSerial.print(roll);
    XBeeSerial.print(" ");
    XBeeSerial.print(alt_kalman);//alt_kalman
    XBeeSerial.print(" ");
    XBeeSerial.print(lat);
    XBeeSerial.print(" ");
    XBeeSerial.println(lon);
  }
  //USBSerial.println(timeElapsed);
}

// simple Kalman filter
// borrowed from http://orlygoingthirty.blogspot.com/2013/06/1-dimensional-kalman-filter-arduino.html?m=1
// adapted from C code by Adrian Boeing, www.adrianboeing.com 
// simplistic Kalman filter for altitude readings
// KalmanFilter kf(0, 0.01, 1.0);
// float avg_err = kf.step(track_err);

KalmanFilter::KalmanFilter(float estimate, float initQ, float initR)
{
  Q = initQ;
  R = initR;

  // initial values for the kalman filter
  x_est_last = 0;
  P_last = 0;

  // initialize with a measurement
  x_est_last = estimate;
}


// add a new measurement, return the Kalman-filtered value
float KalmanFilter::step(float z_measured)
{
  // do a prediction
  x_temp_est = x_est_last;
  P_temp = P_last + R*Q;

  // calculate the Kalman gain
  K = P_temp * (1.0/(P_temp + R));

  // correct
  x_est = x_temp_est + K * (z_measured - x_temp_est); 
  P = (1- K) * P_temp;

  // update our last's
  P_last = P;
  x_est_last = x_est;

  return (x_est);
}

/*
 This routine is run between each
 time loop(), so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent1() {
  while (GPSSerial.available()) {
    // get the new byte:
    char inChar = GPSSerial.read();
      if (gps.encode(inChar)) {   //modified to use miniGPS
        //newdata = true;
        gpsdump(gps);
        // break;  // uncomment to print new data immediately!
      }
  }
}

// ----------------- From TinyGPS example https://www.pjrc.com/teensy/td_libs_TinyGPS.html
void gpsdump(TinyGPS &gps)
{
  //long lat, lon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  //USBSerial.println(lat);
  //USBSerial.print("Lat/Long(10^-5 deg): "); USBSerial.print(lat); USBSerial.print(", "); USBSerial.print(lon); 
  //USBSerial.print(" Fix age: "); USBSerial.print(age); USBSerial.println("ms.");
  
  // On Arduino, GPS characters may be lost during lengthy USBSerial.print()
  // On Teensy, Serial prints to USB, which has large output buffering and
  //   runs very fast, so it's not necessary to worry about missing 4800
  //   baud GPS characters.
  // You will need to uncomment different gps.get or gps.crack lines to be able to use the gps variables
  // Some of these variables will need more NMEA sentences than the (single) GPGGA sentence I am using here
/*
  gps.f_get_position(&flat, &flon, &age);
    USBSerial.print("Lat/Long(float): "); printFloat(flat, 5); USBSerial.print(", "); printFloat(flon, 5);
    USBSerial.print(" Fix age: "); USBSerial.print(age); USBSerial.println("ms.");

  gps.get_datetime(&date, &time, &age);
    USBSerial.print("Date(ddmmyy): "); USBSerial.print(date); USBSerial.print(" Time(hhmmsscc): ");
    USBSerial.print(time);
    USBSerial.print(" Fix age: "); USBSerial.print(age); USBSerial.println("ms.");

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    USBSerial.print("Date: "); USBSerial.print(static_cast<int>(month)); USBSerial.print("/"); 
    USBSerial.print(static_cast<int>(day)); USBSerial.print("/"); USBSerial.print(year);
    USBSerial.print("  Time: "); USBSerial.print(static_cast<int>(hour)); USBSerial.print(":"); 
    USBUSBSerial.print(static_cast<int>(minute)); USBSerial.print(":"); USBSerial.print(static_cast<int>(second));
    USBSerial.print("."); USBSerial.print(static_cast<int>(hundredths));
    USBSerial.print("  Fix age: ");  USBSerial.print(age); USBSerial.println("ms.");

  USBSerial.print("Alt(cm): "); USBSerial.print(gps.altitude()); USBSerial.print(" Course(10^-2 deg): ");
    USBSerial.print(gps.course()); USBSerial.print(" Speed(10^-2 knots): "); USBSerial.println(gps.speed());
    USBSerial.print("Alt(float): "); printFloat(gps.f_altitude()); USBSerial.print(" Course(float): ");
    printFloat(gps.f_course()); USBSerial.println();
    USBSerial.print("Speed(knots): "); printFloat(gps.f_speed_knots()); USBSerial.print(" (mph): ");
    printFloat(gps.f_speed_mph());
    USBSerial.print(" (mps): "); printFloat(gps.f_speed_mps()); USBSerial.print(" (kmph): ");
    printFloat(gps.f_speed_kmph()); USBSerial.println();

  gps.stats(&chars, &sentences, &failed);
    USBSerial.print("Stats: characters: "); USBSerial.print(chars); USBSerial.print(" sentences: ");
    USBSerial.print(sentences); USBSerial.print(" failed checksum: "); USBSerial.println(failed);
*/
}

