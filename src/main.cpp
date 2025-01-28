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
long lastMsg = 0;

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
  float temperature, humidity, distance;
  char text[64];
} struct_mote2sinkMessage;

typedef struct struct_sink2moteMessage {
  int boardId;
  bool bool0;
  char text[64];
} struct_sink2moteMessage;

struct_mote2sinkMessage espNow_incomingReadings;
struct_mote2sinkMessage espNow_lastMotesReadings[MOTE_COUNT] = {};
struct_sink2moteMessage espNow_outgoingMessage;

// ESP-NOW callback for receiving data
void espNowOnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  // Copies the sender MAC address to a string
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  Serial.print("Packet received from MAC = ");
  Serial.print(macStr);
  Serial.print(", length = ");
  Serial.print(len);
  Serial.println(" Bytes");
  Serial.print(espNow_incomingReadings.boardId);
  Serial.println(" Board Id");
  
  // Copy incoming data to the incomingReadings structure
  memcpy(&espNow_incomingReadings, incomingData, sizeof(espNow_incomingReadings));

  // Make a local copy of the last received board data
  espNow_lastMotesReadings[espNow_incomingReadings.boardId] = espNow_incomingReadings;
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
  // MQTT reconnect when needed
  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 500) {
    lastMsg = now;
    for (size_t i = 0; i < MOTE_COUNT; i++) {
      if (espNow_lastMotesReadings[i].timeTag != 0) {
        char lastUpdateString[8];
        char tempString[8];
        char humString[8];
        char distString[8];

        // Switch case on "BoardId"
        switch (espNow_lastMotesReadings[i].boardId) {
        case 0:
          dtostrf(espNow_lastMotesReadings[i].timeTag, 1, 2, lastUpdateString);
          client.publish("esp32/board0/lastUpdate", lastUpdateString);

          dtostrf(espNow_lastMotesReadings[i].temperature, 1, 2, tempString);
          client.publish("esp32/board0/temperature", tempString);

          dtostrf(espNow_lastMotesReadings[i].humidity, 1, 2, humString);
          client.publish("esp32/board0/humidity", humString);
          
          dtostrf(espNow_lastMotesReadings[i].distance, 1, 2, distString);
          client.publish("esp32/board0/distance", distString);

          espNow_lastMotesReadings[i] = {};
          break;

        case 1:
          dtostrf(espNow_lastMotesReadings[i].timeTag, 1, 2, lastUpdateString);
          client.publish("esp32/board1/lastUpdate", lastUpdateString);

          dtostrf(espNow_lastMotesReadings[i].temperature, 1, 2, tempString);
          client.publish("esp32/board1/temperature", tempString);

          dtostrf(espNow_lastMotesReadings[i].humidity, 1, 2, humString);
          client.publish("esp32/board1/humidity", humString);

          dtostrf(espNow_lastMotesReadings[i].distance, 1, 2, distString);
          client.publish("esp32/board0/distance", distString);

          espNow_lastMotesReadings[i] = {};
          break;

        default:
          break;
        }
      }
    }
  }
}
