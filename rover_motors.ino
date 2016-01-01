
// rover_motors - drive the rover motors, accept
// science data (including IMU data) and nav commands
// on Serial3 (hard wired), and execute navigation code
// (machine learning algorithms) to "predict" location
// using GPS location as training data.
//
// Philip R. Moyer
// November 2015

#include <Adafruit_GPS.h>   /* May not be needed */
#include <Rover.h>         /* Rover class and method definitions */

String dataBuffer;          /* Serial read data buffer */
Adafruit_GPS GPS(&Serial2); /* GPS object (currently not working) */
Rover_data roverData;       /* Rover class objects */


// Setup code - runs at system initialization
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial3.begin(9600);
  Serial.println("# Listening for data and commannds from other Mega on Serial3.");
  GPS.begin(9600);      // GPS
  Serial2.begin(9600);  // GPS serial interface
  Serial.println("# Listening for GPS on Serial2.");
}


// Loop() - runs continuously once setup() completes
void loop() {
  // put your main code here, to run repeatedly:
  
  if (Serial3.available() > 0)
  {
    dataBuffer = Serial3.readStringUntil('\n');
    // Serial.write(inByte);
    Serial.print("Master data: ");
    Serial.println(dataBuffer);
  }

  // Read the GPS raw interface
  if (Serial2.available() > 0)
  {
    dataBuffer = Serial2.readStringUntil('\n');
    Serial.println(dataBuffer);
  }

  // Use the GPS library
  if (GPS.newNMEAreceived())
  {
    GPS.parse(GPS.lastNMEA());
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}
