#include <math.h>
//Magnetometer
//compass.h and compass.cpp is written by https://github.com/helscream/HMC5883L_Header_Arduino_Auto_calibration
#include <Wire.h>
#include "compass.h"
//GPS
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

boolean usingInterrupt = false;
void useInterrupt(boolean); 
//When checking for additional coordinates, the vehicle when stop when currentTarget is equal to noOfCoords
int noOfCoords = 3;
//Currently set to drive to three corners of AAU's outdoor basketball field.
float targetLatArray[] = {55.489917, 55.489788, 55.489826};
float targetLongArray[] = {8.446722, 8.446681, 8.446837};

//The connected pins for the DRV8833 motorcontroller (For the current setup)
int motor_pins[] = {8, 9, 10, 11};

//The truth table for the directional movement of the vehicle with the current connected pins.
//          Pin: 8  9  10 11
int forward[] = {0, 1, 0, 1};
int reverse[] = {1, 0, 1, 0};
int left[]    = {1, 0, 0, 1};
int right[]   = {0, 1, 1, 0};
int halt[]    = {0, 0, 0, 0};

float currentLat, currentLong, targetLat, targetLong, desiredHeading;
//CurrentTarget keeps track of the target in the array
//headingThreshold is the amount of degrees the vehicle can be off its desired heading before adjusting the course
//distanceThreshold is the amount of meters away from target coordinate, that when exceeded it will continue towards the next coordinate
int currentTarget = 0, headingThreshold = 4, distanceThreshold = 3;

void setup() {
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  useInterrupt(true);

  //Initializing the pins for the motorcontroller 
  for(int i = 0; i < 4; i++){
    pinMode(motor_pins[i], OUTPUT);
  }
  
  //Offsets and error correction for the compass
  //Found by running the calibration functions supplied by the library
  Wire.begin();
  compass_x_offset = 159.28;
  compass_y_offset = 215.37;
  compass_z_offset = 684.76;
  compass_x_gainError = 1.12;
  compass_y_gainError = 1.13;
  compass_z_gainError = 1.03;
  
  Serial.begin(9600);
  delay(1000);
}


uint32_t timer = millis();
void loop() {
  
  
  Serial.println("----------------------------------------------");
  compass_heading();
  heading += 1.96; //Adding the declination of Esbjerg, Denmark.
  Serial.print ("Current heading/heading: ");
  Serial.print (heading);
  Serial.println(" Degree");
  grabGPSdata();
  adjustCourse();
  Serial.println("----------------------------------------------");
  delay(200);
  

  
}

//--------------------------------------------------------------------------------------------------------------
//End of main loop
//--------------------------------------------------------------------------------------------------------------


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  //Will write the NMEA senteces to serial monitor
  //if (GPSECHO)
  // if (c) UDR0 = c;  
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

//Function to determine the motor outputs
void drive(int pin_array[], int dir_array[]){
  //Loops the passed array so set the drive functions state
  for(int i = 0; i < 4; i++){
    digitalWrite(pin_array[i], dir_array[i]);
      //Debug
      //Serial.print("Pin num: ");Serial.print(pin_array[i]);Serial.print(" - ");Serial.print("Pin State: ");Serial.println(dir_array[i]);
    }
}

void grabGPSdata(){
    // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, grab the current GPS data
  if (millis() - timer > 2000) { 
    timer = millis();
    
    if (GPS.fix) {
      //Uses the GPS library to grab the latitude and longitude in Decimal Degrees
      //But only if there is a GPS fix
      currentLat = GPS.latitudeDegrees;
      currentLong = GPS.longitudeDegrees;
    }else{
      Serial.println("No GPS fix");
    }
  }
      Serial.print("Current Location: ");
      Serial.print(currentLat, 6);
      Serial.print(", "); 
      Serial.println(currentLong, 6);
}

//The function for adjusting the course of the vehicle
//Based on the current heading and desired heading
void adjustCourse(){
  //The distance to the current target coordinate is calculated
  float distance = calculateDistance(currentLat, currentLong, targetLatArray[currentTarget], targetLongArray[currentTarget]);
  //Stops the vehicle when there are no coordinates left
  //If the distance threshold is exceeded, move on to the next target location in the array
  if(currentTarget == noOfCoords){
    Serial.println("No coordinates left in the array.");
    drive(motor_pins, halt);
    return;
  }
  if(distance < distanceThreshold){
    Serial.println("Arrived at the target location.");
    Serial.println("If there are additional coordinates it will continue");
    currentTarget++;

  }
  Serial.print("Distance to next coordinate: ");
  Serial.println(distance);
  //Prints the target location
  Serial.print("Target Location(");Serial.print(currentTarget);Serial.print("): ");
  Serial.print(targetLatArray[currentTarget],6);
  Serial.print(", ");
  Serial.println(targetLongArray[currentTarget],6);
  //Calculates the desired/"optimal" heading based on the current location and target location
  desiredHeading = calculateHeading(currentLat, currentLong, targetLatArray[currentTarget], targetLongArray[currentTarget]);
  Serial.print("Desired heading: ");
  Serial.print(desiredHeading);
  //Find the difference between the current heading and the desired heading
  //And does some correction to find the correct error
  float error = desiredHeading - heading;
  if(error < -180) error += 360;
  if(error > 180) error -= 360;
  Serial.print(" - Error: ");
  Serial.print(error);
  
  //Depending on the error, the vehicle will then adjust its course
  //Small delays for the left and right turns are added to ensure it doesn't oversteer.
  if(abs(error) <= headingThreshold){
    Serial.println(" - On course");
    drive(motor_pins, forward);
  }else if(error < 0){
    Serial.println(" - Adjusting towards the left");
    drive(motor_pins, left);
    delay(50);
    drive(motor_pins, halt);
  }else if (error > 0){
    Serial.println(" - Adjusting towards the right");
    drive(motor_pins, right);
    delay(50);
    drive(motor_pins, halt);
  }else{
    drive(motor_pins, forward);
  }  
}

//Function to calculate the desired heading
float calculateHeading(float currentLat, float currentLong, float targetLat, float targetLong){
  
  //Calculating the x and y arguments needed for the atan2 trig function
  //The heading is based on the target coordinates and the current coordinates
  float x = sin(radians(targetLong-currentLong)) * cos(radians(targetLat));
  float y = cos(radians(currentLat)) * sin(radians(targetLat)) - (sin(radians(currentLat)) * cos(radians(targetLat)) * cos(radians(targetLong-currentLong)));
  float heading = atan2(x, y);
  //If the heading is negative it can't be used, this can be corrected by adding two pi.
  if (heading < 0.0){
    heading += 2*PI;
  }
  
  return degrees(heading); 
}

//Function to calculate the distance between two latitudes and longitudes
float calculateDistance(float currentLat, float currentLong, float targetLat, float targetLong) {
    // Uses the haversine formula to calculate the distance between two points. (http://www.movable-type.co.uk/scripts/latlong.html a variation is found on this page)
    // The distance is returned in meters
    float a = sq((sin((radians(currentLat - targetLat)) / 2.0))) + 
              (cos(radians(currentLat))*cos(radians(targetLat)) * sq((sin((radians(currentLong - targetLong)) / 2.0))));
    float distance = 2.0 * 6371000 * asin (sqrt(a));
    return distance;
 };
