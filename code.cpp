#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <Wire.h>

// Define the GSM module's pins
#define GSM_TX_PIN 4
#define GSM_RX_PIN 5

// Define the GPS module's pins
#define GPS_TX_PIN 6
#define GPS_RX_PIN 7

// Define the flex sensor pin
#define FLEX_PIN A0

// Create SoftwareSerial objects for GSM and GPS modules
SoftwareSerial gsmSerial(GSM_TX_PIN, GSM_RX_PIN);
SoftwareSerial gpsSerial(GPS_TX_PIN, GPS_RX_PIN);

// GPS variables
String latitude;
String longitude;

// Flex sensor variables
int flexThreshold = 600; // Adjust this value based on your sensor readings

// BME280 sensor object for temperature, humidity, and pressure
Adafruit_BME280 bme;

// TinyGPS++ object for GPS parsing
TinyGPSPlus gps;

// Function prototypes
void initializeGSM();
void initializeGPS();
bool getLocation();
void sendNotification(const char* phoneNumber);

void setup() {
  // Start the serial communication with the ESP8266 module
  Serial.begin(9600);

  // Start the serial communication with the GSM module
  gsmSerial.begin(9600);

  // Start the serial communication with the GPS module
  gpsSerial.begin(9600);

  // Initialize the GSM module
  initializeGSM();

  // Initialize the GPS module
  initializeGPS();

  // Initialize the BME280 sensor
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // Read flex sensor value
  int flexValue = analogRead(FLEX_PIN);

  // Check if the flex sensor value exceeds the threshold
  if (flexValue > flexThreshold) {
    // Airbag effect detected, get the GPS location and send notification to emergency services and specific phone number
    if (getLocation()) {
      sendNotification("911"); // Replace with the emergency services number
      sendNotification("+1234567890"); // Replace with the specific phone number
    }
  }

  // Update GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // Uncomment the following line to print raw GPS data
      //Serial.println(gps.encode(gpsSerial.read()));
    }
  }

  // Print BME280 sensor data
  Serial.print("Temperature: ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  Serial.print("Humidity: ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
  Serial.print("Pressure: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  delay(1000);
}

void initializeGSM() {
  // Wait for the GSM module to initialize
  delay(2000);

  // Set up GSM module
  gsmSerial.println("AT+CMGF=1"); // Set SMS mode to text
  delay(1000);
  gsmSerial.println("AT+CNMI=2,2,0,0,0"); // Enable SMS notification
  delay(1000);
  gsmSerial.println("AT+CMGD=1,4"); // Delete all SMS messages
}

void initializeGPS() {
  // Configure GPS module
  gpsSerial.println("$PMTK314,0,1,0,1,0,5");
}

bool getLocation() {
  gpsSerial.println("AT+CGPSINFO"); // Request GPS information

  // Wait for response
  unsigned long timeout = millis();
  while (millis() - timeout < 5000) {
    if (gpsSerial.available()) {
      String response = gpsSerial.readStringUntil('\n');
      if (response.indexOf("$GNRMC") != -1) {
        // Extract latitude and longitude from the GPS response
        int commaIndex1 = response.indexOf(',', 3);
        int commaIndex2 = response.indexOf(',', commaIndex1 + 1);
        int commaIndex3 = response.indexOf(',', commaIndex2 + 1);
        int commaIndex4 = response.indexOf(',', commaIndex3 + 1);
        int commaIndex5 = response.indexOf(',', commaIndex4 + 1);
        int commaIndex6 = response.indexOf(',', commaIndex5 + 1);

        if (commaIndex6 - commaIndex5 > 9) {  // Check if the data is valid
          latitude = response.substring(commaIndex3 + 1, commaIndex4);
          longitude = response.substring(commaIndex5 + 1, commaIndex6);
          return true;
        }
      }
    }
  }

  return false;
}

void sendNotification(const char* phoneNumber) {
  String message = "Emergency! Airbag effect detected. Location: Latitude " + latitude + ", Longitude " + longitude;

  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(phoneNumber);
  gsmSerial.println("\"");
  delay(1000);
  gsmSerial.print(message);
  delay(1000);
  gsmSerial.write(0x1A);
  delay(1000);
}
/// code for gsm 
GSM_TX_PIN and GSM_RX_PIN: The pins used to connect the ESP8266 and GSM module for serial communication.
+1234567890: The recipient's phone number to which the SMS notification will be sent. Please include the country code
#include <SoftwareSerial.h>

// Define the GSM module's pins
#define GSM_TX_PIN 4
#define GSM_RX_PIN 5

// Create a SoftwareSerial object to communicate with the GSM module
SoftwareSerial gsmSerial(GSM_TX_PIN, GSM_RX_PIN);

// GPS variables
String latitude;
String longitude;

void setup() {
  // Start the serial communication with the ESP8266 module
  Serial.begin(9600);
  
  // Start the serial communication with the GSM module
  gsmSerial.begin(9600);
  
  // Wait for the GSM module to initialize
  delay(2000);

  // Set up GSM module
  gsmSerial.println("AT+CMGF=1"); // Set SMS mode to text
  delay(1000);
  gsmSerial.println("AT+CNMI=2,2,0,0,0"); // Enable SMS notification
  delay(1000);
  gsmSerial.println("AT+CMGD=1,4"); // Delete all SMS messages
  
  // Wait for GPS fix
  while (!getLocation()) {
    delay(1000);
  }
  
  // Send the GPS location as a notification
  sendNotification();
}

void loop() {
  // No need for further operations in this example
}

bool getLocation() {
  gsmSerial.println("AT+CGPSINFO"); // Request GPS information
  
  // Wait for response
  unsigned long timeout = millis();
  while (millis() - timeout < 5000) {
    if (gsmSerial.available()) {
      String response = gsmSerial.readStringUntil('\n');
      if (response.indexOf("$GNRMC") != -1) {
        // Extract latitude and longitude from the GPS response
        int commaIndex1 = response.indexOf(',', 3);
        int commaIndex2 = response.indexOf(',', commaIndex1 + 1);
        int commaIndex3 = response.indexOf(',', commaIndex2 + 1);
        int commaIndex4 = response.indexOf(',', commaIndex3 + 1);
        int commaIndex5 = response.indexOf(',', commaIndex4 + 1);
        int commaIndex6 = response.indexOf(',', commaIndex5 + 1);
        
        if (commaIndex6 - commaIndex5 > 9) {  // Check if the data is valid
          latitude = response.substring(commaIndex3 + 1, commaIndex4);
          longitude = response.substring(commaIndex5 + 1, commaIndex6);
          return true;
        }
      }
    }
  }
  
  return false;
}

void sendNotification() {
  String message = "Current location: Latitude " + latitude + ", Longitude " + longitude;
  
  gsmSerial.println("AT+CMGS=\"+1234567890\""); // Replace with your recipient's phone number
  delay(1000);
  gsmSerial.print(message);
  delay(1000);
  gsmSerial.write(0x1A);
  delay(1000);
}
/// code for gsm with emergency contact 
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <Wire.h>

// Define the GSM module's pins
#define GSM_TX_PIN 4
#define GSM_RX_PIN 5

// Define the GPS module's pins
#define GPS_TX_PIN 6
#define GPS_RX_PIN 7

// Define the flex sensor pin
#define FLEX_PIN A0

// Create SoftwareSerial objects for GSM and GPS modules
SoftwareSerial gsmSerial(GSM_TX_PIN, GSM_RX_PIN);
SoftwareSerial gpsSerial(GPS_TX_PIN, GPS_RX_PIN);

// GPS variables
String latitude;
String longitude;

// Flex sensor variables
int flexThreshold = 600; // Adjust this value based on your sensor readings

// BME280 sensor object for temperature, humidity, and pressure
Adafruit_BME280 bme;

// TinyGPS++ object for GPS parsing
TinyGPSPlus gps;

// Function prototypes
void initializeGSM();
void initializeGPS();
bool getLocation();
void sendNotification(const char* phoneNumber);

void setup() {
  // Start the serial communication with the ESP8266 module
  Serial.begin(9600);

  // Start the serial communication with the GSM module
  gsmSerial.begin(9600);

  // Start the serial communication with the GPS module
  gpsSerial.begin(9600);

  // Initialize the GSM module
  initializeGSM();

  // Initialize the GPS module
  initializeGPS();

  // Initialize the BME280 sensor
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // Read flex sensor value
  int flexValue = analogRead(FLEX_PIN);

  // Check if the flex sensor value exceeds the threshold
  if (flexValue > flexThreshold) {
    // Airbag effect detected, get the GPS location and send notification to emergency services and specific phone number
    if (getLocation()) {
      sendNotification("911"); // Replace with the emergency services number
      sendNotification("+1234567890"); // Replace with the specific phone number
    }
  }

  // Update GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // Uncomment the following line to print raw GPS data
      //Serial.println(gps.encode(gpsSerial.read()));
    }
  }

  // Print BME280 sensor data
  Serial.print("Temperature: ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  Serial.print("Humidity: ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
  Serial.print("Pressure: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  delay(1000);
}

void initializeGSM() {
  // Wait for the GSM module to initialize
  delay(2000);

  // Set up GSM module
  gsmSerial.println("AT+CMGF=1"); // Set SMS mode to text
  delay(1000);
  gsmSerial.println("AT+CNMI=2,2,0,0,0"); // Enable SMS notification
  delay(1000);
  gsmSerial.println("AT+CMGD=1,4"); // Delete all SMS messages
}

void initializeGPS() {
  // Configure GPS module
  gpsSerial.println("$PMTK314,0,1,0,1,0,5");
}

bool getLocation() {
  gpsSerial.println("AT+CGPSINFO"); // Request GPS information

  // Wait for response
  unsigned long timeout = millis();
  while (millis() - timeout < 5000) {
    if (gpsSerial.available()) {
      String response = gpsSerial.readStringUntil('\n');
      if (response.indexOf("$GNRMC") != -1) {
        // Extract latitude and longitude from the GPS response
        int commaIndex1 = response.indexOf(',', 3);
        int commaIndex2 = response.indexOf(',', commaIndex1 + 1);
        int commaIndex3 = response.indexOf(',', commaIndex2 + 1);
        int commaIndex4 = response.indexOf(',', commaIndex3 + 1);
        int commaIndex5 = response.indexOf(',', commaIndex4 + 1);
        int commaIndex6 = response.indexOf(',', commaIndex5 + 1);

        if (commaIndex6 - commaIndex5 > 9) {  // Check if the data is valid
          latitude = response.substring(commaIndex3 + 1, commaIndex4);
          longitude = response.substring(commaIndex5 + 1, commaIndex6);
          return true;
        }
      }
    }
  }

  return false;
}

void sendNotification(const char* phoneNumber) {
  String message = "Emergency! Airbag effect detected. Location: Latitude " + latitude + ", Longitude " + longitude;

  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(phoneNumber);
  gsmSerial.println("\"");
  delay(1000);
  gsmSerial.print(message);
  delay(1000);
  gsmSerial.write(0x1A);
  delay(1000);
}