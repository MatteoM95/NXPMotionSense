// Full orientation sensing using NXP's advanced sensor fusion algorithm.
//
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// added altitude reading - this requires new sparkfun mpl3115 library to be added to arduino ide

#include <NXPMotionSense.h>
#include <SparkFunMPL3115A2.h>
#include <Wire.h>
#include <EEPROM.h>
#include "kalman.h"
//Create an instance of the object
MPL3115A2 myPressure;
NXPMotionSense imu;
NXPSensorFusion filter;
KalmanFilter kf(0, 0.008, 10); //was (0, .01, 1.0) 

void setup() {
  Serial.begin(9600); // Not using the usb serial ... but kept this in
  //Begin HW serial
  Serial1.begin(230400); //set to highest baud rate for xbee pro module
  //XBees need to be programmed with the above baud setting. XBee Pro S1 max 115200
  //XBee pro SC2 max at 230400 baud. Need to make sure firmware in xbee is set to 802.15.4 TH PRO setting
  imu.begin();
  filter.begin(100);
  
  myPressure.begin(); // Get mpl3115 sensor online
  //Configure the sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(2); // Set Oversample to the recommended 128 - 
  // sampling intervals of 1=6 ms , 2=10, 4=18, 8=34, 16=66, 32=130, 64=258, and 128=512 
  //oversampling set to 128 takes half a second to perform. Even when set to 8 there is some lag for the orientation. 
  //Seems to be ok when set to 2. There is pretty good filtering with the simplified Kalman filter
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;
   
  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
    // Update the SensorFusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    int alt = (int) myPressure.readAltitudeFt(); // gets rid of decimals in reading for faster transmition 
    float alt_kalman = kf.step(alt); //filter noise from altimeter sensor, and get readings to settle

    // print the heading, pitch and roll to the xbee serial port
    // kept decimals in for these variables because I liked the smoother movement in the processing display
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial1.print("O: ");
    Serial1.print(heading);
    Serial1.print(" ");
    Serial1.print(pitch);
    Serial1.print(" ");
    Serial1.print(roll);
    Serial1.print(" ");
    Serial1.println(alt_kalman);
  }
}

// simple Kalman filter
// borrowed from http://orlygoingthirty.blogspot.com/2013/06/1-dimensional-kalman-filter-arduino.html?m=1
// adapted from C code by Adrian Boeing, www.adrianboeing.com 
// simplistic Kalman filter for encoder readings
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
