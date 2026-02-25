// BLE library
#include <ArduinoBLE.h>

// WiFi libraries
#include <WiFiNINA.h>

#include <ArduinoHttpClient.h>

// Pulse library
#include <PulseSensorPlayground.h>

// Location library
#include <ArduinoJson.h>


// BLE defines
#define SERVICE_UUID "88ebc83b-9f5f-4710-8312-d29cdc3c3a90"
#define BPM_UUID    "12345678-1234-5678-1234-56789abcde02"
#define CPR_UUID     "12345678-1234-5678-1234-56789abcde04"

// -------- Variables --------
// BLE variables 
BLEDevice peripheral;
BLECharacteristic bpmChar;
BLECharacteristic cprChar;
bool bleConnected = false;
bool scanningStarted = false; 
bool pulseEnable = true; // TBD commented
bool bleWasConnected = false;

// WiFi variables 
const char* ssid = "Your_SSID";		//For security reasons, we masked this ssid
const char* pass = "Your_PASSWORD"; //For security reasons, we masked this password

// IFTTT variables 
String eventName = "P-A"; // IFTTT event name
String iftttKey = "c0nyVcQnxqVTbxHHh9oTO";
int iftttTrigger = 1;

// Pulse variables
int myBPM = 0;
volatile bool bpmReceived = false;
unsigned long lastBpmSendTime = 0;
const unsigned long BPM_SEND_INTERVAL = 5000; // 2 seconds
const int PulseWire = A0;
int Threshold = 520;
unsigned long lastBleSendTime = 0;      // track last BLE send  //NPK - additional
const unsigned long BLE_SEND_INTERVAL = 500; // 500 ms  // track last BLE send  //NPK - additinal
bool cprActive = false;
bool bpmWrite = false;
bool httpBusy = false;
int Signal;
int BPM = 0;
unsigned long lastBeatTime = 0;
unsigned long beatInterval = 0;
bool beatDetected = false;
String status = ""; // BPM status
String lastStatus = "";
PulseSensorPlayground pulseSensor;

// HTTPS API endpoint for Pulse
const char* serverPulse = "https://pe1px9j88i.execute-api.us-east-1.amazonaws.com/bpm";

// Location variables
double lat = 0.0;
double lon = 0.0;


void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!BLE.begin()) {
    Serial.println("Failed to start BLE!");
    while (1);
  }

  Serial.println("BLE central started scanning...");
  BLE.scanForUuid(SERVICE_UUID);

}

void loop() {
  readPulse();
  BLE.poll();

  // -------- BLE CONNECT --------
  if (!bleConnected) {
    BLEDevice device = BLE.available();

    if (device) {
      Serial.print("Found Peripheral device: ");
      Serial.println(device.localName());

      BLE.stopScan();

      if (device.connect()) {
        Serial.println("Connected to peripheral");

        if (device.discoverAttributes()) {
          Serial.println("Attributes discovered");

          bpmChar = device.characteristic(BPM_UUID);
          cprChar = peripheral.characteristic(CPR_UUID);

          if (bpmChar) {
            peripheral = device;
            bleConnected = true;
            Serial.println("Characteristic found. Ready to write.");
          } else {
            Serial.println("Characteristic not found!");
            device.disconnect();
            BLE.scanForUuid(SERVICE_UUID);
          }
        }
      }
    }
    return;
  }

  // BLE LOST
  if (!peripheral.connected()) {
    Serial.println("BLE Disconnected. Restarting scan...");
    bleConnected = false;
    BLE.scanForUuid(SERVICE_UUID);
    return;
  }

  // -------- PULSE SENSOR --------
  // BPM calculation
  if (pulseEnable && (millis() - lastBpmSendTime >= BPM_SEND_INTERVAL)) {
    myBPM = BPM;
    if (beatDetected) {
      Serial.print("myBPM: ");
      Serial.println(myBPM);
    }
    // Send data            
    lastBpmSendTime = millis(); //send BPM via BLE and HTTPS every interval
    
    // ------- Checks -------
    if (myBPM < 30) {
        status = "dangerously low";
    } 
    else if (myBPM > 120) {
        status = "dangerously high";
    }
    else if ((myBPM >= 30 && myBPM <= 60) || (myBPM >= 100 && myBPM <= 120)) {
        status = "warning";
    }
    else {
        status = "normal";
    }
    
    if ((status == "dangerously low" || status == "dangerously high") && myBPM != 0) {
      Serial.println("iftttTrigger now");
      iftttTrigger = 2;
    }

    //send BPM via BLE to Nano BLE Sense
    uint8_t hrValue[2];
    hrValue[0] = (myBPM >> 8) & 0xFF;
    hrValue[1] = myBPM & 0xFF;
    
    if (bpmChar.writeValue(hrValue, 2)) {
      Serial.print("BLE message Sent: ");
      Serial.println(myBPM);
    } else {
      Serial.println("BLE Write failed!");
    }
    
    // -------- Disconnect BLE before WiFi --------
    peripheral.disconnect();
    bleConnected = false;
    delay(200);

    Serial.println("bpmAPI request sent");
    bpmAPI(myBPM, status);
    Serial.println("bpmAPI completed");

    // Resume BLE
    Serial.println("Restarting BLE stack...");
    BLE.end();
    delay(300);
    if (!BLE.begin()) {
      Serial.println("BLE restart failed!");
      while (1);
    }
    delay(300);
    BLE.scanForUuid(SERVICE_UUID);
    lastBpmSendTime = millis();  // After WiFi completes and BLE restarts, reset the timer
    Serial.println("BLE scanning restarted again after WiFi disconnect");

    BLEDevice device1 = BLE.available();
    if (device1) {
      Serial.println("Device found! - Peripheral");
    }

    // BLE notification for CPR
    if (cprChar.valueUpdated()) {
      cprActive = cprChar.value()[0];
    }
  }
}

// -------- User defined functions --------
void bpmAPI(int bpmValue, String status) {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, pass);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(200);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println("\nWiFi Connected");

  WiFiSSLClient client; // secure client
  HttpClient http(client, "pe1px9j88i.execute-api.us-east-1.amazonaws.com", 443);

  String json = "{\"device_id\":\"bpm_monitor\",\"BPM\":" + String(bpmValue) + ",\"status\":\"" + status + "\"}";

  http.beginRequest();
  http.post("/bpm");
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();

  int statusCode = http.responseStatusCode();
  Serial.print("IFTTT Response Code: ");
  Serial.println(statusCode);

  http.stop();    // stop HTTP

  // get location and send IFTTT call
  if ((status == "dangerously low" || status == "dangerously high") && myBPM != 0) {
    location();
    sendIFTTTCall(status);
  }

  WiFi.disconnect();  // disconnect wifi
  WiFi.end();
  delay(200);
  Serial.println("WiFi Disconnected");
}



void readPulse() {
  Signal = analogRead(PulseWire);

  if (Signal > Threshold && !beatDetected) {
    beatDetected = true;
    unsigned long now = millis();
    beatInterval = now - lastBeatTime;
    lastBeatTime = now;

    if (beatInterval > 300) {   // ignore noise (<200 BPM)
      BPM = 60000 / beatInterval;
    }
  }

  if (Signal < Threshold - 20) {
    beatDetected = false;
  }
}

void location() {
  const char* server = "3svmn1ujs9.execute-api.us-east-1.amazonaws.com";
  const int port = 443; // HTTPS port

  WiFiSSLClient sslClient;   // for HTTPS
  HttpClient client = HttpClient(sslClient, server, port);

  Serial.println("Requesting location...");

  client.beginRequest();
  client.get("/location"); // API path
  client.sendHeader("Accept", "application/json");
  client.endRequest();

  int statusCode = client.responseStatusCode();
  String response = client.responseBody();
  Serial.print("Length: ");
  Serial.println(response.length());
  Serial.print("Full Response: ");
  Serial.println(response);

  response.trim(); // Remove extra whitespace

  Serial.print("Status code: ");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response);

  // Parse JSON
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, response);

  if (error) {
    Serial.print("JSON parse failed: ");
    Serial.println(error.c_str());
  } else {
    double lat = doc["lat"];
    double lon = doc["lon"];
    Serial.print("Latitude: ");
    Serial.println(lat, 6);
    Serial.print("Longitude: ");
    Serial.println(lon, 6);
  }

  client.stop();

  delay(10000); // Fetch every 10 seconds

}

void sendIFTTTCall(String status) {
  WiFiSSLClient sslClient;

  const char* host = "maker.ifttt.com";
  const int port = 443;

  HttpClient httpClient = HttpClient(sslClient, host, port);

  String url = "/trigger/" + eventName + "/with/key/" + iftttKey +
               "?value1=" + status +
               "&value2=" + String(lat, 6) +
               "&value3=" + String(lon, 6);

  Serial.println("Sending IFTTT request...");

  httpClient.get(url);

  int statusCode = httpClient.responseStatusCode();
  String response = httpClient.responseBody();

  Serial.print("IFTTT HTTPS Status code: ");
  Serial.println(statusCode);

  Serial.println("Response:");
  Serial.println(response);

  httpClient.stop();

  Serial.println("IFTTT REQUEST SENT");
}
