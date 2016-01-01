
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
Sd2Card card;                 // SD card data object
SdVolume volume;              // Volume on SD card
SdFile root;                  // Root of filesystem on data logger SD card
File fileHandle;              // Generic file object for data files on the SD card

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

  Serial.println("# CPU B Initializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("# CPU B initialization failed. Things to check:");
    Serial.println("# CPU B * is a card inserted?");
    Serial.println("# CPU B * is your wiring correct?");
    Serial.println("# CPU B * did you change the chipSelect pin to match your shield or module?");
    return;
  } else {
    Serial.println("# CPU B Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.print("\n# CPU B Card type: ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("# CPU B Could not find FAT16/FAT32 partition.\n# CPU B Make sure you've formatted the card");
    return;
  }


  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("\n# CPU B Volume type is FAT");
  Serial.println(volume.fatType(), DEC);
  Serial.println();

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
  Serial.print("# CPU B Volume size (bytes): ");
  Serial.println(volumesize);
  Serial.print("# CPU B Volume size (Kbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("# CPU B Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);


  Serial.println("\n# CPU B Files found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);

  // Initialize the SD card for writing and reading
  if (!SD.begin(chipSelect))
  {
    Serial.println("# CPU B SD card initialization failed.");
  }
  else
  {
    Serial.println("# CPU B data logger SD card ready.");
  }
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
  if (changedFlag && (0 < roverData.getGPSmonth()))
  {
    if (debug)
    {
      Serial.println("# CPU B updating data files on SD card");
    }
    // NOTE: the SD library only supports one data file to be opened at a time, for either reading or writing.
    // Need to figure out how to append to an existing file....
    // 
    // File "science.csv" contians all science data with timestamp, including IMU data

    fileHandle = SD.open("science.tsv", FILE_WRITE);
    if (fileHandle)
    {
      // Get GPS timestamp and use it for these data
      fileHandle.print(roverData.getGPSyear()); fileHandle.print("-"); fileHandle.flush();
      fileHandle.print(roverData.getGPSmonth()); fileHandle.print("-"); fileHandle.flush();
      fileHandle.print(roverData.getGPSday()); fileHandle.print(" "); fileHandle.flush();
      fileHandle.print(roverData.getGPShours()); fileHandle.print(":"); fileHandle.flush();
      fileHandle.print(roverData.getGPSminutes()); fileHandle.print(":");  fileHandle.flush();
      fileHandle.print(roverData.getGPSseconds()); fileHandle.flush();
      fileHandle.print("\t"); fileHandle.flush();
      
      // Accelerometer
      fileHandle.print(roverData.getAccelX()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getAccelY()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getAccelZ()); fileHandle.print("\t"); fileHandle.flush();
  
      // Gyroscope
      fileHandle.print(roverData.getGyroX()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getGyroY()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getGyroZ()); fileHandle.print("\t"); fileHandle.flush();
  
      // Magnetometer
      fileHandle.print(roverData.getMagX()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getMagY()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getMagZ()); fileHandle.print("\t"); fileHandle.flush();
  
      // Atomic values
      fileHandle.print(roverData.getTempA()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getTempB()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getTempC()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getLight()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getLightVisible()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getLightIR()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getLightUV()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getLightUVindex()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getRH()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.println(roverData.getPressure());  fileHandle.flush();
      fileHandle.close();
    }
    else
    {
      Serial.println("# CPU B Error writing science data to data logger SD card");
    }
    
    // File "gps.csv" contains timestamp and GPS lat, long, alt, speed, heading

    fileHandle = SD.open("gps.tsv", FILE_WRITE);
    if (fileHandle)
    {
      // GPS
      // Get GPS timestamp and use it for these data
      fileHandle.print(roverData.getGPSyear()); fileHandle.print("-"); fileHandle.flush();
      fileHandle.print(roverData.getGPSmonth()); fileHandle.print("-"); fileHandle.flush();
      fileHandle.print(roverData.getGPSday()); fileHandle.print(" "); fileHandle.flush();
      fileHandle.print(roverData.getGPShours()); fileHandle.print(":"); fileHandle.flush();
      fileHandle.print(roverData.getGPSminutes()); fileHandle.print(":");  fileHandle.flush();
      fileHandle.print(roverData.getGPSseconds()); fileHandle.flush();
      fileHandle.print("\t"); fileHandle.flush();
      // data
      fileHandle.print(roverData.getGPSlat()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getGPSlong()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getGPSalt()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getGPSspeed()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getGPSheading()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.print(roverData.getGPSfix()); fileHandle.print("\t"); fileHandle.flush();
      fileHandle.println(roverData.getGPSsats()); fileHandle.flush();
      fileHandle.close();
    }
    else
    {
      Serial.println("# CPU B Error writing GPS data to data logger SD card");
    }
    
    // File "drivetrain.csv" contains data about the rover's power levels to motors, calculated wheel rotation, and time intervals
    // File "location.csv" contains the location relative to the start based on science (IMU) data and movmement algorithms 
    //      - brute force estimate of location
    // File "predict.csv" contains the AI location prediction based on training data
    // File "location_predict.csv" contains the AI location prediction based on live data, relative to start location
    
  }
}

