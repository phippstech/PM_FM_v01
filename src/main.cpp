/* Source Code for the Voltek Power Manager Board
 * Developer: Cody Phipps
 * Firmware Version: 00
 * 
 */ 
#include <Wire.h>
#include <Arduino.h>
#include <ESPUI.h>
#include <WiFi.h> // Include the WiFi library
#include <HTTPClient.h> // Include the HTTPClient library
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SLAVE_ADDRESS_U7 8 // Address of the slave device U7
#define SLAVE_ADDRESS_U10 9 // Address of the slave device U10

float C1 = 0;       // variable to store the calculated current value of the current monitor input position one
float C2 = 0;       // variable to store the calculated current value of the current monitor input position two
float C3 = 0;       // variable to store the calculated current value of the current monitor input position three
float C4 = 0;       // variable to store the calculated current value of the current monitor input position four
float C5 = 0;       // variable to store the calculated current value of the current monitor input position five
float C6 = 0;       // variable to store the calculated current value of the current monitor input position six

int F1 = 0;    // variable to store the read value of the status of the position one fault line (digital high or low)
int F2 = 0;    // variable to store the read value of the status of the position two fault line (digital high or low)
int F3 = 0;    // variable to store the read value of the status of the position three fault line (digital high or low)
int F4 = 0;    // variable to store the read value of the status of the position four fault line (digital high or low)
int F5 = 0;    // variable to store the read value of the status of the position five fault line (digital high or low)
int F6 = 0;    // variable to store the read value of the status of the position six fault line (digital high or low)

// BLE Service and Characteristic UUIDs
#define SERVICE_UUID "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-210987654321"

// BLE Server and Characteristic
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;

// WiFi credentials
const char* ssid = "ESP32-AP";
const char* password = "12345678";

// Status label ID
uint16_t statusLabel;

void setup() 
{
  Wire.begin(); // Join the I2C bus as a master
  Serial.begin(9600); // Start the Serial communication
  ESPUI.setVerbosity(Verbosity::Verbose); // Enable verbose output so you see the files in LittleFS
  delay(500); // Delay to allow Serial Monitor to start after a reset
  Serial.println(F("\nPreparing filesystem with ESPUI resources"));
  ESPUI.prepareFileSystem();  // Copy across current version of ESPUI resources
  Serial.println(F("Done, files..."));
  ESPUI.list(); // List all files on LittleFS, for info

  // Set up Access Point
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  // Initialize ESPUI
  ESPUI.begin("Voltek Power Manager");
  statusLabel = ESPUI.label("Status: Connected to AP", ControlColor::Turquoise, "Status");

  // Initialize BLE
  BLEDevice::init("Voltek Power Manager");
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
}

void loop() 
{
  // Request data from U7
  Wire.requestFrom(SLAVE_ADDRESS_U7, 200); // Request 200 bytes from the slave device
  String jsonData_U7 = "";
  while(Wire.available()) 
  { // While there is data available
    char c = Wire.read(); // Read a character
    jsonData_U7 += c; // Append the character to the JSON string
  }

  if (jsonData_U7.length() > 0) 
  {
    StaticJsonDocument<200> doc_U7;
    DeserializationError error = deserializeJson(doc_U7, jsonData_U7);
    if (!error) 
    {
      C4 = doc_U7["Current Reading line 4"];
      C5 = doc_U7["Current Reading line 5"];
      C6 = doc_U7["Current Reading line 6"];
      F4 = doc_U7["Fault line 4"];
      F5 = doc_U7["Fault line 5"];
      F6 = doc_U7["Fault line 6"];
      Serial.print("U7 - Current Measurement 4: ");
      Serial.println(C4);
      Serial.print("U7 - Current Measurement 5: ");
      Serial.println(C5);
      Serial.print("U7 - Current Measurement 6: ");
      Serial.println(C6);
    } 
    else 
    {
      Serial.print("deserializeJson() failed for U7: ");
      Serial.println(error.c_str());
    }
  }

  // Request data from U10
  Wire.requestFrom(SLAVE_ADDRESS_U10, 200); // Request 200 bytes from the slave device
  String jsonData_U10 = "";
  while(Wire.available()) 
  { // While there is data available
    char c = Wire.read(); // Read a character
    jsonData_U10 += c; // Append the character to the JSON string
  }

  if (jsonData_U10.length() > 0) 
  {
    StaticJsonDocument<200> doc_U10;
    DeserializationError error = deserializeJson(doc_U10, jsonData_U10);
    if (!error) 
    {
      C1 = doc_U10["Current Reading line 1"];
      C2 = doc_U10["Current Reading line 2"];
      C3 = doc_U10["Current Reading line 3"];
      F1 = doc_U10["Fault line 1"];
      F2 = doc_U10["Fault line 2"];
      F3 = doc_U10["Fault line 3"];
      Serial.print("U10 - Current Measurement 1: ");
      Serial.println(C1);
      Serial.print("U10 - Current Measurement 2: ");
      Serial.println(C2);
      Serial.print("U10 - Current Measurement 3: ");
      Serial.println(C3);
    } 
    else 
    {
      Serial.print("deserializeJson() failed for U10: ");
      Serial.println(error.c_str());
    }
  }

  // Send a state to the slave devices
  char state[4] = "111"; // Turn all positions on
  Wire.beginTransmission(SLAVE_ADDRESS_U7); // Begin a transmission to the slave device U7
  Wire.write((const uint8_t *)state, sizeof(state) - 1); // Write the state to the slave device
  Wire.endTransmission(); // End the transmission

  Wire.beginTransmission(SLAVE_ADDRESS_U10); // Begin a transmission to the slave device U10
  Wire.write((const uint8_t *)state, sizeof(state) - 1); // Write the state to the slave device
  Wire.endTransmission(); // End the transmission

  // Update BLE characteristic with the latest data
  StaticJsonDocument<400> doc;
  doc["C1"] = C1;
  doc["C2"] = C2;
  doc["C3"] = C3;
  doc["C4"] = C4;
  doc["C5"] = C5;
  doc["C6"] = C6;
  doc["F1"] = F1;
  doc["F2"] = F2;
  doc["F3"] = F3;
  doc["F4"] = F4;
  doc["F5"] = F5;
  doc["F6"] = F6;
  String output;
  serializeJson(doc, output);
  pCharacteristic->setValue(output.c_str());
  pCharacteristic->notify();

  delay(10000); // Send a request every 10 seconds
}