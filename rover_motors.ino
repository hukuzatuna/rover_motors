
// rover_motors - drive the rover motors, accept
// science data (including IMU data) and nav commands
// on Serial3 (hard wired), and execute navigation code
// (machine learning algorithms) to "predict" location
// using GPS location as training data.
//
// Philip R. Moyer
// November 2015

// Preprocessor directives

#include <SPI.h>            // SPI library to support data logger
#include <SD.h>             // SD card data logger library
#include <Rover.h>          // Rover class and method definitions
#include "TinyGPS++.h"      // GPS handling library

// Constants

const bool debug = false;   // Whether or not to be verbose on Serial
const int chipSelect = 53;  // Data logger chip select pin

// Global variables

char dataBuffer[255];         // Serial read data buffer
Rover_data roverData;         // Rover class objects
TinyGPSPlus gps;              // GPS device

// Functions

void printRoverScienceData()
{
  // Accelerometer
  Serial.println("---------------------------------------------");
  Serial.print("Accel X: "); Serial.print(roverData.getAccelX()); Serial.println(" m/s^2");
  Serial.print("Accel Y: "); Serial.print(roverData.getAccelY()); Serial.println(" m/s^2");
  Serial.print("Accel Z: "); Serial.print(roverData.getAccelZ()); Serial.println(" m/s^2");
  
  // Gyroscope
  Serial.print("Gyro X: "); Serial.print(roverData.getGyroX()); Serial.println(" rad/sec");
  Serial.print("Gyro Y: "); Serial.print(roverData.getGyroY()); Serial.println(" rad/sec");
  Serial.print("Gyro Z: "); Serial.print(roverData.getGyroZ()); Serial.println(" rad/sec");
  
  // Magnetometer
  Serial.print("Mag X: "); Serial.print(roverData.getMagX()); Serial.println(" gauss");
  Serial.print("Mag Y: "); Serial.print(roverData.getMagY()); Serial.println(" gauss");
  Serial.print("Mag Z: "); Serial.print(roverData.getMagZ()); Serial.println(" gauss");
  
  // Atomic values
  Serial.print("Temp A: "); Serial.print(roverData.getTempA()); Serial.println(" C");
  Serial.print("Temp B: "); Serial.print(roverData.getTempB()); Serial.println(" C");
  Serial.print("Temp C: "); Serial.print(roverData.getTempC()); Serial.println(" C");
  Serial.print("Light: "); Serial.print(roverData.getLight()); Serial.println(" lux");
  Serial.print("Visible light: "); Serial.println(roverData.getLightVisible());
  Serial.print("IR light: "); Serial.println(roverData.getLightIR());
  Serial.print("UV light: "); Serial.println(roverData.getLightUV());
  Serial.print("UV index: "); Serial.println(roverData.getLightUVindex());
  Serial.print("Humidity: "); Serial.print(roverData.getRH()); Serial.println(" %");
  Serial.print("Pressure: "); Serial.print(roverData.getPressure()); Serial.println(" mb");

  // GPS
  Serial.print("Lat: "); Serial.print(roverData.getGPSlat()); Serial.println(" deg.");
  Serial.print("Long: "); Serial.print(roverData.getGPSlong()); Serial.println(" deg.");
  Serial.print("Alt: "); Serial.print(roverData.getGPSalt()); Serial.println(" m");
  Serial.print("Time: "); Serial.print(roverData.getGPShours()); Serial.print(":");
  Serial.print(roverData.getGPSminutes()); Serial.print(":"); Serial.println(roverData.getGPSseconds());
  Serial.print("Date: "); Serial.print(roverData.getGPSday()); Serial.print("-");
  Serial.print(roverData.getGPSmonth()); Serial.print("-"); Serial.println(roverData.getGPSyear());
  Serial.print("Speed: "); Serial.print(roverData.getGPSspeed()); Serial.println(" kmph");
  Serial.print("Heading: "); Serial.print(roverData.getGPSheading()); Serial.println(" deg");
  Serial.print("Fix: "); Serial.println(roverData.getGPSfix());
  Serial.print("Sats: "); Serial.println(roverData.getGPSsats());
  Serial.println("---------------------------------------------");
}

// Setup code - runs at system initialization
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("# CPU B Startup sequence initiated");
  Serial3.begin(115200);
  Serial.println("# CPU B Listening for data and commannds from Arduino A on Serial3.");
  Serial2.begin(9600);  // GPS serial interface
  Serial.println("# CPU B Listening for GPS on Serial2.");
}


// Loop() - runs continuously once setup() completes
void loop() {
  // put your main code here, to run repeatedly:

  bool changedFlag = false;
  
  while (Serial3.available() > 0)
  {
    dataBuffer[0] = 0;
    Serial3.readBytesUntil('\n', dataBuffer, sizeof(dataBuffer));
    if (0 == strncmp("COMM CHECK", dataBuffer, 10))
    {
      // Communications check from Arduino A
      Serial3.println("OK");
      Serial.println("# CPU B Communications established with CPU A");
    }
    else
    {
      if (0 == strncmp("LIDAR", dataBuffer, 5))
      {
        // Not sure why I'm sending the LIDAR data. This system doesn't do anything with it.
        if (debug)
        {
          Serial.println("# CPU B received LIDAR data");
        }
      }
      else
      {
        // Otherwise we have science data
        if (debug)
        {
          Serial.print("# CPU B master data: ");
          Serial.println(dataBuffer);
        }
        if (debug)
        {
          Serial.println("# CPU B storing science data in memory object");
        }
        roverData.parseMasterData(dataBuffer);    // Parse and store master data from Arduino A
        changedFlag = true;
        if (debug)
        {
          printRoverScienceData();
        }
      }
    }
  }

  // Read the GPS raw interface
  while (Serial2.available() > 0)
  {
    gps.encode(Serial2.read());
  }
  if (gps.location.isUpdated())
  {
    // gps object has updated its location data - we've moved (or the GPS has wandered)
    roverData.setGPSlat(gps.location.lat());
    roverData.setGPSlong(gps.location.lng());
    roverData.setGPSalt(gps.altitude.meters());
    roverData.setGPShours(gps.time.hour());
    roverData.setGPSminutes(gps.time.minute());
    roverData.setGPSseconds((double)gps.time.second() + gps.time.centisecond());
    roverData.setGPSday(gps.date.day());
    roverData.setGPSmonth(gps.date.month());
    roverData.setGPSyear(gps.date.year());
    roverData.setGPSsats(gps.satellites.value());
    roverData.setGPSspeed(gps.speed.kmph());
    roverData.setGPSheading(gps.course.deg());
    roverData.setGPSfix(true);
    if (debug)
    {
      printRoverScienceData();
    }
    changedFlag = true;
  }

  // Write data to SD card:
  if (changedFlag)
  {
    if (debug)
    {
      Serial.println("# CPU B updating data files on SD card");
    }
    // File "science" contians all science data with timestamp
    // File "train_x" contains timestamp and IMU data, wheel radius, rpm of loaded motors, time since last data write or motor change
    // File "train_y" contains timestamp and GPS lat, long, alt, speed, heading
    // File "location_a" contains the location relative to the start based on train_x data and movmement algorithms
    // File "predict_train" contains the AI location prediction based on training data
    // File "location_predict" contains the AI location prediction based on live data, relative to start location
  }
}

