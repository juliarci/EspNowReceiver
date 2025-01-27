/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Adapted by A.Combes 2025
*********/

#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Replace with your network credentials
const char* ssid = "iot";
const char* password = "iotisis;";

// MQTT broker IP address
const char* mqtt_server = "192.168.3.151";
WiFiClient espClient;
PubSubClient client(espClient);

#define MOTE_COUNT 2
uint8_t moteAddress[MOTE_COUNT][6] = {
  {0xEC, 0x62, 0x60, 0x5A, 0x78, 0x4B},
  {}
};
esp_now_peer_info_t peerInfo[MOTE_COUNT];

// Data structures for ESP-NOW communication
typedef struct struct_mote2sinkMessage {
  int boardId;
  int readingId;
  int timeTag;
  float temperature, humidity;
  char text[64];
} struct_mote2sinkMessage;

typedef struct struct_sink2moteMessage {
  int boardId;
  bool bool0;
  char text[64];
} struct_sink2moteMessage;

struct_mote2sinkMessage espNow_lastMotesReadings[MOTE_COUNT] = {};
struct_sink2moteMessage espNow_outgoingMessage;

// ESP-NOW callback for receiving data
void espNowOnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&espNow_lastMotesReadings, incomingData, sizeof(struct_mote2sinkMessage));
  Serial.printf("Data received from board %d\n", espNow_lastMotesReadings->boardId);
  // Log data
  Serial.printf("Reading ID: %d, TimeTag: %d\n", espNow_lastMotesReadings->readingId, espNow_lastMotesReadings->timeTag);
}

// ESP-NOW callback for sending data
void espNowOnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("Packet sent to %02x:%02x:%02x:%02x:%02x:%02x, Status: %s\n",
                mac_addr[0], mac_addr[1], mac_addr[2],
                mac_addr[3], mac_addr[4], mac_addr[5],
                status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Send data to a mote
void espNowSendDataToMote(struct_sink2moteMessage message) {
  esp_err_t result = esp_now_send(moteAddress[message.boardId], (uint8_t*)&message, sizeof(message));
  Serial.println(result == ESP_OK ? "Data sent successfully" : "Error sending data");
}

// MQTT callback
void mqttCallback(char* topic, byte* message, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)message[i];
  Serial.printf("MQTT Message: %s on topic %s\n", msg.c_str(), topic);

  // Handle commands based on MQTT topic
  if (String(topic) == "esp32/board0/output") {
    espNow_outgoingMessage.boardId = 0;
    espNow_outgoingMessage.bool0 = (msg == "on");
    espNowSendDataToMote(espNow_outgoingMessage);
  }
}

// Reconnect to MQTT broker
void mqttReconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT Broker!");
      client.subscribe("esp32/board0/output");
    } else {
      Serial.print("Failed to connect. Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

// Setup Wi-Fi
void setupWiFi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Wi-Fi connected");
}

// Initialize ESP-NOW
void setupEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(espNowOnDataRecv);
  esp_now_register_send_cb(espNowOnDataSent);

  for (int i = 0; i < MOTE_COUNT; i++) {
    memcpy(peerInfo[i].peer_addr, moteAddress[i], 6);
    peerInfo[i].channel = 0;
    peerInfo[i].encrypt = false;
    if (esp_now_add_peer(&peerInfo[i]) != ESP_OK) {
      Serial.println("Failed to add peer");
    }
  }
}

// Setup function
void setup() {
  Serial.begin(115200);

  setupWiFi();
  setupEspNow();

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
}

// Main loop
void loop() {
  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();

  // Publish ESP-NOW data to MQTT
  for (int i = 0; i < MOTE_COUNT; i++) {
    if (espNow_lastMotesReadings[i].timeTag != 0) {
      char dataStr[32];
      snprintf(dataStr, sizeof(dataStr), "%.2f", espNow_lastMotesReadings[i].temperature);
      client.publish("esp32/mote/data", dataStr);
    }
  }

  delay(1000);
}
