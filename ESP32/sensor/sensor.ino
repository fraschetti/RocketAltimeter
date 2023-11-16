//#define UseBTLE
#define UseWiFi
//#define UseWiFiLR
#define UseSerial

#define SENSOR_ID 1;

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <LinkedList.h>
#include <math.h>

/* Begin BTLE */
#ifdef UseBTLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif
/* End BTLE */

/* Begin ESP-NOW */
#ifdef UseWiFi
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#endif
/* End ESP-NOW */

/* Begin BMP390 */
//Working I2C = SDA=8, SCL=9, Blue=GPIO7, Yellow=GPIO9
//#define I2C_SDA 7
//#define I2C_SCL 9

#define I2C_SDA 20
#define I2C_SCL 21


#define SEALEVELPRESSURE_HPA (1013.25)

LinkedList<float> heightWindow = LinkedList<float>();
const int WINDOW_EVALUATION_INTERVAL = 15 * 1000;
unsigned long nextWindowEvaluation = -1;
const float STABLE_HEIGHT_STD_DEV = 0.1;

float firstAltitudeOffset = -1;
float currentAltitudeOffset = 0;  // Start high so any real values are the new low

TwoWire I2CBME = TwoWire(0);
Adafruit_BMP3XX bmp;

void bmp390Init() {
  I2CBME.begin(I2C_SDA, I2C_SCL, 100000);
  if (!bmp.begin_I2C(0x77, &I2CBME)) {  // hardware I2C mode, can pass in address & alt Wire
#ifdef UseSerial
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
#endif
    while (250)
      ;
  }

  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);  //Disabled

  //What looked good?
  //BMP3_OVERSAMPLING_2X, BMP3_IIR_FILTER_COEFF_3, BMP3_ODR_25_HZ

  /* More noise but faster updates
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  */

  bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);  //2x was responsive
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);     //1 looked good, 3 also looked good
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);

  /* Less noise but slower updates
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  */
}

void generateWindowStats(float &std_dev, float &mean, float &minVal) {
  int size = heightWindow.size();
  float sum = 0;
  float sq_sum = 0;
  minVal = heightWindow.get(0);

  for (int i = 0; i < size; i++) {
    float data = heightWindow.get(i);
    sum += data;
    sq_sum += data * data;
    if (data < minVal)
      minVal = data;  // update the min value
  }
  mean = sum / size;
  // The standard deviation is the square root of the mean of the squares minus the square of the mean.
  std_dev = sqrt(sq_sum / size - mean * mean);
  if (!isnan(std_dev)) {
    std_dev = 0.0;
  }

#ifdef UseSerial
  /*
  Serial.print("DataPoints:");
  Serial.print(size);
  Serial.print(",StdDev:");
  Serial.print(std_dev);
  Serial.print(",Mean:");
  Serial.print(mean);
  Serial.print(",Min:");
  Serial.println(minVal);
  */
#endif
}

void evaulateHeightWindow() {
#ifdef UseSerial
  Serial.println("Evaluating recent height values...");
#endif

  int size = heightWindow.size();
#ifdef UseSerial
  Serial.print("Height window data points: ");
  Serial.println(size);
#endif
  if (size > 50) {
    float sd, mean, min;
    generateWindowStats(sd, mean, min);

    //TODO print min/max?/avg to determine if there's an issue here
    if (sd < STABLE_HEIGHT_STD_DEV) {
      Serial.println("Mabybe set alt offset");
      Serial.print("currentAltitudeOffset: ");
      Serial.println(currentAltitudeOffset);
      Serial.print("firstAltitudeOffset: ");
      Serial.println(firstAltitudeOffset);
      Serial.print("(currentAltitudeOffset - firstAltitudeOffset): ");
      float deltaFromFirstOffset = fabs(currentAltitudeOffset - firstAltitudeOffset);
      Serial.println(deltaFromFirstOffset);
      //TODO change this to be a delta from the current offset instead of a fixed "first"?
      //Allow for slight adjustments +/-
      if (currentAltitudeOffset == 0 || (deltaFromFirstOffset) < 2) {
        Serial.println("Set!");
        currentAltitudeOffset = mean;

#ifdef UseSerial
        Serial.print("Setting altitude offset: ");
        Serial.println(currentAltitudeOffset);
#endif

        //Once we have our first stable calibration, others need to be at a similar height
        if (firstAltitudeOffset == -1) {
          Serial.println("Setting firstAltitudeOffset to currentAltitudeOffset");
          firstAltitudeOffset = currentAltitudeOffset;
        }
      } else {
        Serial.println("Skipped set. Criteria not met :(");
      }
    }
  } else {
    Serial.println("Skipping calibration. Not enough data points");
  }

  Serial.println();
  Serial.println();

  heightWindow.clear();

  // Set next window evaluation time
  nextWindowEvaluation = millis() + WINDOW_EVALUATION_INTERVAL;
}

float readHeight() {
  if (!bmp.performReading()) {
#ifdef UseSerial
    Serial.println("Failed to perform reading :(");
#endif
    return 0;
  }

  float measuredAltitudeMeters = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  heightWindow.add(measuredAltitudeMeters);  //Add it to the window

  unsigned long now = millis();
  if (nextWindowEvaluation == -1 || now >= nextWindowEvaluation) {
    evaulateHeightWindow();
  }

  float adjustedAltitudeMeters;
  if (measuredAltitudeMeters < currentAltitudeOffset) {
    adjustedAltitudeMeters = 0;
  } else {
    adjustedAltitudeMeters = measuredAltitudeMeters - currentAltitudeOffset;
  }

#ifdef UseSerial
/*
  Serial.print("MeasuredAltMeters:");
  Serial.print(measuredAltitudeMeters);
  Serial.print(",MeasuredAltFeet:");
  Serial.print(measuredAltitudeMeters * 3.28084);
  Serial.print(",AdjustedAltMeters:");
  Serial.print(adjustedAltitudeMeters);
  Serial.print(",CurrentAltitudeOffset:");
  Serial.print(currentAltitudeOffset);
  */
#endif

  float adjustedAltitudeFeet = 3.28084 * adjustedAltitudeMeters;

#ifdef UseSerial
/*
  Serial.print(",AdjustedAltitudeFeet:");
  Serial.println(adjustedAltitudeFeet);
  */
#endif

  return adjustedAltitudeFeet;
}
/* End BMP390 */

/* Begin BLE */
#ifdef UseBTLE
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLEDescriptor *pDescr;
BLE2902 *pBLE2902;

int bleServerCount = 0;

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    bleServerCount++;
#ifdef UseSerial
    Serial.println("New server connected!");
    Serial.print("Servers: ");
    Serial.println(bleServerCount);
#endif

    //TODO move to a timer after 500ms?
    BLEDevice::startAdvertising();
  };

  void onDisconnect(BLEServer *pServer) {
    bleServerCount--;

#ifdef UseSerial
    Serial.println("Server disconnected!");
    Serial.print("Servers: ");
    Serial.println(bleServerCount);
#endif
  }
};

void bleInit() {
  // Create the BLE Device
  BLEDevice::init("RocketSensor");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);

  // Create a BLE Descriptor

  pDescr = new BLEDescriptor((uint16_t)0x2901);
  pDescr->setValue("A very interesting variable");
  pCharacteristic->addDescriptor(pDescr);

  pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);
  pCharacteristic->addDescriptor(pBLE2902);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
#ifdef UseSerial
  Serial.println("Waiting a client connection to notify...");
#endif
}

void bleLoop() {
  if (bleServerCount > 0) {
    float height = readHeight();

    pCharacteristic->setValue(height);
    pCharacteristic->notify();
  }

  delay(50);  // careful or the bluetooth stack will go into congestion
}
#endif
/* End BLE */

/* Begin ESP-NOW */
#ifdef UseWiFi
const int wifiChannel = 4;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int id;  // must be unique for each sender board
  float height;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create peer interface
esp_now_peer_info_t serverInfo;

void scanForServer() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, wifiChannel);  // Scan only on one channel

  // reset on each scan
  memset(&serverInfo, 0, sizeof(serverInfo));

  bool serverFound = false;

#ifdef UseSerial
  Serial.println("");
#endif
  if (scanResults == 0) {
#ifdef UseSerial
    Serial.println("No WiFi devices in AP Mode found");
#endif
  } else {
#ifdef UseSerial
    Serial.print("Found ");
    Serial.print(scanResults);
    Serial.println(" devices ");
#endif
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

#ifdef UseSerial
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(SSID);
      Serial.print(" (");
      Serial.print(RSSI);
      Serial.print(")");
      Serial.println("");
#endif

      delay(10);
      // Check if the current device name contains 'RocketServer'
      if (SSID.indexOf("RocketServer") >= 0) {
// SSID of interest
#ifdef UseSerial
        Serial.println("Found a server.");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
#endif

        // Get BSSID => Mac Address of the Server
        int mac[6];
        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5])) {
          for (int ii = 0; ii < 6; ++ii) {
            serverInfo.peer_addr[ii] = (uint8_t)mac[ii];
          }
        }

        serverInfo.channel = wifiChannel;  // pick a channel
        serverInfo.encrypt = false;        // no encryption

        serverFound = true;
        break;
      }
    }
  }

#ifdef UseSerial
  if (serverFound) {
    Serial.println("Server Found, processing..");
  } else {
    Serial.println("Server Not Found, trying again.");
  }
#endif

  // clean up ram
  WiFi.scanDelete();
}

// Check if the server is already paired with this device.
// If not, pair the server
bool addServerPeer() {
  if (serverInfo.channel == wifiChannel) {
    deletePeer();

#ifdef UseSerial
    Serial.print("Server Status: ");
#endif
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(serverInfo.peer_addr);
    if (exists) {
// Server already paired.
#ifdef UseSerial
      Serial.println("Already Paired");
#endif
      return true;
    } else {
      // Server not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&serverInfo);
      if (addStatus == ESP_OK) {
// Pair success
#ifdef UseSerial
        Serial.println("Pair success");
#endif
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
// How did we get so far!!
#ifdef UseSerial
        Serial.println("ESPNOW Not Init");
#endif
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
#ifdef UseSerial
        Serial.println("Invalid Argument");
#endif
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
#ifdef UseSerial
        Serial.println("Peer list full");
#endif
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
#ifdef UseSerial
        Serial.println("Out of memory");
#endif
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
#ifdef UseSerial
        Serial.println("Peer Exists");
#endif
        return true;
      } else {
#ifdef UseSerial
        Serial.println("Not sure what happened");
#endif
        return false;
      }
    }
  } else {
// No server found to process
#ifdef UseSerial
    Serial.println("No Server found to process");
#endif
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(serverInfo.peer_addr);

#ifdef UseSerial
  Serial.print("Server Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
#endif
}

unsigned int wifiSendFailures = 0;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#ifdef UseSerial
//Serial.print("Last Packet Send Status: ");
//Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
#endif

  if (status == ESP_NOW_SEND_SUCCESS) {
    wifiSendFailures = 0;
  } else {
    wifiSendFailures++;

#ifdef UseSerial
    Serial.print("WiFi send failure: #");
    Serial.println(wifiSendFailures);
#endif

    delay(50);

    // After 10 failures, let's look for a new server
    if (wifiSendFailures >= 100) {
      deletePeer();
      // reset server info
      memset(&serverInfo, 0, sizeof(serverInfo));
    }
  }
}

void espNowInit() {
#ifdef UseSerial
  Serial.print("ESP Board WiFi MAC Address:  ");
  Serial.println(WiFi.macAddress());
#endif

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

#ifdef UseWiFiLR
  int a = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#ifdef UseSerial
  Serial.println(a);
#endif
#endif


  // Init ESP-NOW
  ESP_ERROR_CHECK(esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE));
  if (esp_now_init() != ESP_OK) {
#ifdef UseSerial
    Serial.println("Error initializing ESP-NOW");
#endif
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
}

void espNowLoop() {
  // If we don't have a sever, loop for one
  if (serverInfo.channel != wifiChannel) {
    scanForServer();
    if (addServerPeer()) {
#ifdef UseSerial
      Serial.println("Successfully paired server");
#endif
    } else {
#ifdef UseSerial
      Serial.println("Failed to pair server");
#endif

      // reset server definition
      memset(&serverInfo, 0, sizeof(serverInfo));
    }
  }

  // If we have a server now, try to transmit
  if (serverInfo.channel == wifiChannel) {
    // Set values to send
    myData.id = SENSOR_ID;
    myData.height = readHeight();

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(serverInfo.peer_addr, (uint8_t *)&myData, sizeof(myData));

#ifdef Debug
#ifdef UseSerial
    Serial.print("Sent height:");
    Serial.print(myData.height);
    Serial.print(",Success:");
    Serial.println(result == ESP_OK);
#endif
#endif

    //delay(50); //TODO CHRIS enable once this is fast on battery?
  } else {
    // Don't loop too fast looking for APs
    delay(1000);
  }
}
#endif
/* End ESP-NOW */

void setup() {
  //TODO CHRIS
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

#ifdef UseSerial
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
#endif

  bmp390Init();

#ifdef UseBTLE
  bleInit();
#endif

#ifdef UseWiFi
  espNowInit();
#endif
}

void loop() {
#ifdef UseBTLE
  bleLoop();
#endif

#ifdef UseWiFi
  espNowLoop();
#endif
}