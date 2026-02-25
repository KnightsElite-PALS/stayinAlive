/*
This code contains pulse sensor to API gateway + lambda + dynamodb and IFTTT
with deviceid, timestamp, pulse status, and location coordinates, and speaker,
Edge Impuse Y/N detection, and BLE communication between Nano 33 BLE Sense and ESP32

Expected serial output :

Central connected
Track 1 started
Track finished: 1
DNR check finished
Starting inference...
Microphone started successfully
Detected DNR label: yes
Track 2 started
BPM received: 13
Pulse Status: dangerously low
Starting CPR instructions

*/

// ------- Libraries -------
// Pulse sensor libraries
#include <ArduinoHttpClient.h>
#include <PulseSensorPlayground.h>

// Speaker libraries
#include <DFRobotDFPlayerMini.h>
#include <Arduino.h>

// BLE + Edge Impulse libraries
#include <ArduinoBLE.h>
#include <StayinAlive_Y_N_inferencing.h>
#include <PDM.h>

// Pulse sensor define
#define USE_ARDUINO_INTERRUPTS false  

// ------- Variables -------
// Pulse sensor variables
int myBPM = 0;
volatile bool newBpmReceived = false;
String status = "";
String lastStatus = "";

// Speaker variables
HardwareSerial &mp3Serial = Serial1;
DFRobotDFPlayerMini player;
bool played4 = false;
bool played2 = false;
bool cprActive = false;
bool lastCprStatus = false;
bool dnrCheckPlayed = false;
bool dnrCheckFinished = false;
bool recoveryActive = false;

// BLE variables
bool bleWasConnected = false;
BLEService testService("88ebc83b-9f5f-4710-8312-d29cdc3c3a90");
BLECharacteristic bpmChar("12345678-1234-5678-1234-56789abcde02", BLEWrite | BLERead, 2);
BLECharacteristic cprChar("12345678-1234-5678-1234-56789abcde04", BLENotify, 1);

// --- EI variables ---
//bool startInference = false;
bool inferenceStarted = false;
bool inferenceRunning = false;
bool inferenceCompleted = false;

// Edge Impulse inference buffers
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;
static inference_t inference;
static int16_t sampleBuffer[2048];
static bool debug_nn = false;
static bool stopInference = false;

String detectedDNRLabel = "";

static void pdm_data_ready_inference_callback(void) {
    int bytesAvailable = PDM.available();
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if (inference.buf_ready == 0) {
        for (int i = 0; i < bytesRead >> 1; i++) {
            inference.buffer[inference.buf_count++] = sampleBuffer[i];
            if (inference.buf_count >= inference.n_samples) {
                inference.buf_ready = 1;
                break;
            }
        }
    }
}

static bool microphone_inference_start(uint32_t n_samples) {
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
    if (!inference.buffer) return false;

    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    PDM.onReceive(&pdm_data_ready_inference_callback);
    PDM.setBufferSize(4096);

    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        Serial.println("Failed to start PDM!");
        return false;
    }
    PDM.setGain(127);
    Serial.println("Microphone started successfully");

    return true;
}

static bool microphone_inference_record(void) {
    inference.buf_ready = 0;
    inference.buf_count = 0;

    while (inference.buf_ready == 0) {
      BLE.poll();
      delay(10);
    }
    return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0;
}

static void microphone_inference_end(void) {
    PDM.end();
    free(inference.buffer);
}

void setup() {
  Serial.begin(9600);
  delay(1000);  // wait for Serial Monitor to connect

  // Speaker setup
  mp3Serial.begin(9600);
  if (!player.begin(mp3Serial)) {
    Serial.println("DFPlayer not found!");
    while (true);  // stop if missing
  }
  player.volume(20);

  // BLE setup
  if (!BLE.begin()) {
    Serial.println("BLE failed to start!");
    while (true);
  }
  BLE.setLocalName("NanoSense");
  BLE.setAdvertisedService(testService);
  testService.addCharacteristic(bpmChar);
  testService.addCharacteristic(cprChar);
  BLE.addService(testService);

  BLE.advertise(); // continuously advertise while no central is connected
  Serial.println("Advertising service...");

} // setup END

void loop() {
  BLE.poll(); // keeps BLE connection alive
  checkPlayerEvents();

  if (BLE.connected()) {
    // Start DNR check once
    if (!dnrCheckPlayed && !bleWasConnected) {
        Serial.println("Central connected");
        bleWasConnected = true;
        dnrCheck();   // play track 1
    }

    if (dnrCheckFinished && !inferenceRunning && !inferenceCompleted) {
        inferenceRunning = true;
        Serial.println("Starting inference...");
        bleInferencing();
    }

  } else {
      if (bleWasConnected) {
        Serial.println("Central disconnected");
        BLE.advertise();
        Serial.println("Advertising restarted");
        bleWasConnected = false;
      }
  }


  //TBD to comment this block
  if (!BLE.connected()) {
    BLE.advertise(); 
    //Serial.println("Advertising service... - in loop()");
  }


  if (bpmChar.written()) {
    uint8_t data[2];
    bpmChar.readValue(data, 2); 
    Serial.print("bpmChar value: ");
    Serial.println(bpmChar);
    
    myBPM = (data[0] << 8) | data[1];
    newBpmReceived = true;

    Serial.print("BPM received - myBPM: ");
    Serial.println(myBPM);
  }

  // ------- Calculate BPM -------
  if (newBpmReceived) {
    newBpmReceived = false;

    // Checks
    String newStatus;
    if (myBPM < 30) {
      newStatus = "dangerously low";
    }
    else if (myBPM > 120) {
      newStatus = "dangerously high";
    }
    else if ((myBPM >= 30 && myBPM <= 60) || (myBPM >= 100 && myBPM <= 120)) {
      newStatus = "warning";
    }
    else {
      newStatus = "normal";
    }

    // React only when status changes
    if (newStatus != lastStatus) {
      Serial.println("Pulse Status: " + newStatus);

      // Start CPR
      if ((newStatus == "dangerously low" || newStatus == "dangerously high") && 
            detectedDNRLabel == "yes" && !cprActive) {
        Serial.println("Starting CPR instructions");
        player.stop();  // stop playing any audio
        played4 = false;
        cprInstructions();      // call CPR instructions function
        cprActive = true;
        recoveryActive = false;
      }
      // Recovery
      if (newStatus == "normal" && cprActive && detectedDNRLabel == "yes") {
          recoveryAudio();
      }
      lastStatus = newStatus;
    }
    status = newStatus;

    // Send cprActive status to ESP
    if (cprActive != lastCprStatus) {
      uint8_t cpr = cprActive ? 1 : 0; // convert bool to byte
      if (BLE.connected()) {
        cprChar.writeValue(&cpr, 1);
      }
      lastCprStatus = cprActive;
      Serial.print("CPR Active Status: ");
      Serial.println(cprActive);
    }
  }
}

// ------- User defined functions -------
// CPR instructions
void dnrCheck() {
  if (!dnrCheckPlayed) {
    player.playMp3Folder(1);
    Serial.println("Track 1 started");
    dnrCheckPlayed = true;
  }
}

void cprInstructions() {
  player.stop();
  player.playMp3Folder(3);
  Serial.println("Track 3 started");
  cprActive = true;
  played4 = false;
}

void recoveryAudio() {
  Serial.println("BPM recovered, playing recovery audio");
  player.stop();    // stop playing any audio
  player.playMp3Folder(5);    // play recovery audio
  recoveryActive = true;
  cprActive = false;
  played4 = false;
  resetInference(); 
}

// EI inferencing
void bleInferencing() {
    if (inferenceCompleted) return;
    if (!stopInference) {
        run_classifier_init();
        if (!microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT)) {
          Serial.println("Failed to start microphone");
           inferenceRunning = false; 
          return;
        }
        microphone_inference_record();
        signal_t signal;
        signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
        signal.get_data = &microphone_audio_signal_get_data;
        ei_impulse_result_t result = {0};
        EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
        if (r != EI_IMPULSE_OK) {
            microphone_inference_end();
            return;
        }
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (result.classification[ix].value >= 0.98f) {
                detectedDNRLabel = result.classification[ix].label;
                Serial.println("Detected DNR label: " + detectedDNRLabel);
                inferenceCompleted = true;
                inferenceRunning = false;
                microphone_inference_end();
                if (!played2) {
                    player.playMp3Folder(2);
                    played2 = true;
                }
                break;
            }
        }
    }
}

void checkPlayerEvents() {
    if (player.available()) {
        uint8_t type = player.readType();
        uint16_t value = player.read();

        if (type == DFPlayerPlayFinished) {
            Serial.print("Track finished: ");
            Serial.println(value);
            if (value == 1) {
              dnrCheckFinished = true;
              Serial.println("DNR check finished");           
            }
            if (value == 3 && cprActive) {
                player.playMp3Folder(4);
                played4 = true;
            }
            // Loop song
            if (value == 4 && cprActive) {
              Serial.println("Looping Stayin' Alive");
              player.playMp3Folder(4);
            }
        }
    }
}

void resetInference() {

/*  // TBD 02/22 block comment added
  stopInference = false;
  //startInference = true;
  //inferenceDone = false;
  inferenceStarted = false;

*/

  // TBD 02/22 added below set and serial.println 
  inferenceRunning = false;
  inferenceCompleted = false;
  stopInference = false;
  dnrCheckFinished = false;
  dnrCheckPlayed = false;
  detectedDNRLabel = "";

  Serial.println("Inference reset");
}
