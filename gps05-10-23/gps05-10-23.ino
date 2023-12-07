arduino#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(3, 4); // RX, TX
Adafruit_GPS GPS(&gpsSerial);

// Define variables to store data
String latitude, longitude, altitude, speed, time, date;

void setup() {
  Serial.begin(115200);
  GPS.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Request RMC (recommended minimum) and GGA (fix data) sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // Set update rate to 1 Hz
  delay(1000);
}

void loop() {
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // Parse the latest NMEA sentence received
      return;

    if (GPS.fix) {
      latitude = String(GPS.latitudeDegrees, 6);
      longitude = String(GPS.longitudeDegrees, 6);
      altitude = String(GPS.altitude);
      speed = String(GPS.speed);
      time = String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds);
      date = String(GPS.day) + "/" + String(GPS.month) + "/" + String(GPS.year);

      Serial.println("Latitude: " + latitude);
      Serial.println("Longitude: " + longitude);
      Serial.println("Altitude: " + altitude + " meters");
      Serial.println("Speed: " + speed + " knots");
      Serial.println("Time: " + time);
      Serial.println("Date: " + date);
      Serial.println("-----------------------------");
    } else {
      Serial.println("No Fix");
      Serial.println("-----------------------------");
    }
  }
}