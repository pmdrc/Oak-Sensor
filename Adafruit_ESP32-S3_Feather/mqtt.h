#include "PubSubClient.h"
#include <ArduinoJson.h>

#define sensorNumber  1

String mqttName = "Room sensor" + String(sensorNumber);
String stateTopic = "home/rooms/" + String(sensorNumber) + "/state";

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
PubSubClient client(wifiClient);

void sendMQTTTemperatureDiscoveryMsg() {
  String discoveryTopic = "homeassistant/sensor/room_sensor_" + String(sensorNumber) + "/temperature/config";

  DynamicJsonDocument doc(1024);
  char buffer[256];

  doc["name"] = "Room " + String(sensorNumber) + " Temperature";
  doc["stat_t"]   = stateTopic;
  doc["unit_of_meas"] = "°C";
  doc["dev_cla"] = "temperature";
  doc["frc_upd"] = true;
  doc["val_tpl"] = "{{ value_json.temperature|default(0) }}";

  size_t n = serializeJson(doc, buffer);

  client.publish(discoveryTopic.c_str(), buffer, n);
}

void sendMQTTHumidityDiscoveryMsg() {
  String discoveryTopic = "homeassistant/sensor/room_sensor_" + String(sensorNumber) + "/humidity/config";

  DynamicJsonDocument doc(1024);
  char buffer[256];

  doc["name"] = "Room " + String(sensorNumber) + " Humidity";
  doc["stat_t"]   = stateTopic;
  doc["unit_of_meas"] = "%";
  doc["dev_cla"] = "humidity";
  doc["frc_upd"] = true;
  doc["val_tpl"] = "{{ value_json.humidity|default(0) }}";

  size_t n = serializeJson(doc, buffer);

  client.publish(discoveryTopic.c_str(), buffer, n);
}

void sendMQTTMoistureDiscoveryMsg() {
  String discoveryTopic = "homeassistant/sensor/room_sensor_" + String(sensorNumber) + "/battery/config";

  DynamicJsonDocument doc(1024);
  char buffer[256];

  doc["name"] = "Room " + String(sensorNumber) + " Battery";
  doc["stat_t"]   = stateTopic;
  doc["unit_of_meas"] = "%";
  doc["dev_cla"] = "battery"; 
  doc["frc_upd"] = true;
  doc["val_tpl"] = "{{ value_json.battery|default(0) }}";

  size_t n = serializeJson(doc, buffer);

  client.publish(discoveryTopic.c_str(), buffer, n);
}

