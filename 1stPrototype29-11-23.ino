#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
sensors_event_t event; 
Servo throttleServo;
Servo steerServo;
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO false
float declinationAngle = 0.034;
float longi;
float lati;
double waypointLat = 52.24838;
double waypointLong = 4.43470;
double dir = 0;
float headingDegrees = 0;
bool approx(double current,  double goal, double margin){
  if(current <= goal + margin && current >= goal - margin){
    return true;
  }
  return false;
}
double getMag(){
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  heading += declinationAngle;
  if(heading < 0){
    heading += 2*PI;
  } 
  if(heading > 2*PI){
    heading -= 2*PI;
  }
  
  return  heading * 180/M_PI;
}
double calcBearing(double curLat, double curLon,double desLat,double  desLon)
 {
  double bearing = atan2((sin((desLon - curLon)*M_PI/180) *cos(desLat*M_PI/180)),((cos(curLat*M_PI/180)*sin(desLat*M_PI/180))
  - (sin(curLat*M_PI/180)*cos(desLat*M_PI/180)*cos((desLon-curLon)*M_PI/180))));
  if(bearing < 0){
    bearing += 2*PI;
  }
  if(bearing > 2*PI){
    bearing -= 2*PI;
  }
  return bearing;
 }

void setup() {
   while(!mag.begin())
  {
     
    delay(5);
  }
  throttleServo.attach(9);
  throttleServo.write(1500);
  steerServo.attach(11);
  steerServo.write(100);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
}
void Left(){
   steerServo.write(50);
  delay(50);
  throttleServo.write(1535);
  delay(750);
  throttleServo.write(1500);
  steerServo.write(100);
  delay(1200);
  return;
}
void Right(){
  steerServo.write(150);
  delay(50);
  throttleServo.write(1530);
  delay(2300);
  throttleServo.write(1500);
  steerServo.write(100);
  delay(2000);
  return;
}
void loop() {
  if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
     return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  if (GPS.fix) {
    
  }
  headingDegrees = getMag();
  
  
  
  if(!approx(GPS.latitudeDegrees, waypointLat, 0.00005) || 
      !approx(GPS.longitudeDegrees, waypointLong, 0.00005)){
         dir = calcBearing(GPS.latitudeDegrees, GPS.longitudeDegrees, waypointLat, waypointLong);
      }
  while(!approx(headingDegrees, dir, 10)){
    Left();
    for(int i = 0; i<= 3; i++){
  headingDegrees = getMag();
    }
  delay(200);
    }
  
  }
  /*else if(headingDegrees > 180 && headingDegrees < 320){
    Right();
  }else{
    while(1){}
 }*/
   
  
 //Right();


//}
