//#define UseWiFiLR

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const int wifiChannel = 4;

typedef struct struct_message {
  int id;  // must be unique for each sender board
  float height;
} struct_message;

void initESPNow() {
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);

#ifdef UseWiFiLR
  Serial.println("Enabling long range mode...");
  int a = esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR);
  //Serial.println(a);
#endif

  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());

  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
}

void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.print("Last Packet Recv from: ");
  //Serial.println(macStr);

  struct_message* msg = (struct_message*)data;

  Serial.print("Sensor ID:");
  Serial.print(msg->id);
  Serial.print(",Height:");
  Serial.println(msg->height);
}

// config AP SSID
void configDeviceAP() {
  Serial.print("ESP Board WiFi MAC Address:  ");
  Serial.println(WiFi.macAddress());

  const char* SSID = "RocketServer";
  bool result = WiFi.softAP(SSID, "Server1_1_Password", wifiChannel, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL ");
    Serial.println(WiFi.channel());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting RocketServer...");

  initESPNow();
}


// This is the Arduino main loop function.
void loop() {
}