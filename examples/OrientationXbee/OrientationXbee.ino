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
//Create an instance of the object
MPL3115A2 myPressure;
NXPMotionSense imu;
NXPSensorFusion filter;

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
  myPressure.setOversampleRate(2); // Set Oversample to the recommended 128 - I set this to 1 to not slow down the orientation readings
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
    Serial1.println(alt);
  }
}

