#include <WiFi.h>

#include "mqtt.h"
#include "sensors.h"
#include "secrets.h"

#include "Adafruit_NeoPixel.h"
#include "Adafruit_LC709203F.h"

#define NUMPIXELS      1
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */
#define WAIT_FOR_WIFI  5
#define WAIT_FOR_MQTT  5
#define DEBUGME        1

// Battery Gauge Monitor
Adafruit_LC709203F lc;
// How many internal neopixels do we have? some boards have more than one!
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Variables defined in secrets.h
// char ssid[]     = "MySSID";
// char password[] = "WiFiPass";
// const char* mqttServer = "192.168.10.3"; // The IP of your MQTT broker
// const int mqttPort = 1883;
// const char* mqttUser = "homeassistant";
// const char* mqttPassword = "Secret";


String mqttName = "Room sensor" + String(sensorNumber);
String stateTopic = "home/rooms/" + String(sensorNumber) + "/state";

WiFiClient wifiClient;
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
PubSubClient client(wifiClient);

void printme(const char* text) {
#if defined(DEBUGME) 
  Serial.print(text);
#endif
}

void setup() 
  {
  bool status;
  // Turn on any internal power switches for TFT, NeoPixels, I2C, etc!
  enableInternalPower();
  // set color to red 
  pixels.fill(0xFFFFFF);
  pixels.show();
  delay(50);
  #if defined(DEBUGME) 
  Serial.begin(115200);
  while (!Serial) {
    delay(100); // wait for serial port to connect. Needed for native USB port only
  }
  #endif
  if (!lc.begin()) {
    printme("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!\n");
  }
  else {
    lc.setPackSize(LC709203F_APA_1000MAH);
    //lc.setAlarmVoltage(3.8);
  }
  delay(10);
  if ((status=aht.begin())==false) {
    printme("Could not find AHT? Check wiring\n");
    delay(1000);
  }
  else {
    printme("AHT10 or AHT20 found\n");
  }

  // WIFI ============================== We start by connecting to a WiFi network
  pixels.fill(0xFF00FF); // LED MAGENTA
  pixels.show();
  printme("n\Connecting to ");
  printme(ssid);
  WiFi.begin(ssid, password);
  uint32_t t1 = millis();
  bool wifi_timeout=false;
  while (WiFi.status() != WL_CONNECTED) {
    printme(".");
    #if defined(DEBUGME)   
    Serial.print(WiFi.status());
    #endif
    if (millis() - t1 > WAIT_FOR_WIFI * 1000)
    {
      printme("Timeout connecting to WiFi.\n");
      wifi_timeout = true;
      break;
    }
    delay(20);
  }

  if (wifi_timeout)
  {
    disableInternalPower();
    delay(100);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    printme("Going to sleep now");
    esp_deep_sleep_start(); 
  }
  else
  {
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    #if defined(DEBUGME)
    printme("\n");
    printme("WiFi connected\n");
    printme("IP address: ");
    Serial.println(WiFi.localIP());
    #endif
  }

  // MQTT =================================================================
  pixels.fill(0x0000FF); // set LED to blue
  pixels.show();
  client.setServer(mqttServer, mqttPort);
  printme("Connecting to MQTT");
  uint32_t t2 = millis();
  bool mqtt_timeout=false;
  while (!client.connected())
  {
    printme(".");
    if (millis() - t2 > WAIT_FOR_MQTT * 1000)
    {
      printme("Timeout connecting to MQTT.\n");
      mqtt_timeout = true;
      break;
    }
    delay(20);
  }
  if (mqtt_timeout)
  {
    disableInternalPower();
    delay(100);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    printme("Going to sleep now");
    esp_deep_sleep_start(); 
  }
  else
  {
    if (client.connect(mqttName.c_str(), mqttUser, mqttPassword)) 
    {
      printme("Connected to MQTT\n");
        sendMQTTTemperatureDiscoveryMsg();
        sendMQTTHumidityDiscoveryMsg();
        sendMQTTMoistureDiscoveryMsg();
      } 
      else 
      {
        printme("failed with state ");
        #if defined(DEBUGME)
        Serial.println(client.state());
        #endif
      }
  }

// READ SENSORs ============================================== 
  sensors_event_t humidity, temp;

  pixels.fill(0x00FF00);   // turn green
  pixels.show();
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  #if defined(DEBUGME) 
  Serial.print("Temperature: "); 
  Serial.print(temp.temperature);
  Serial.println(" degrees C");
  Serial.print("Humidity: "); 
  Serial.print(humidity.relative_humidity); 
  Serial.println("% rH");

  Serial.print("Batt_Voltage:");
  Serial.print(lc.cellVoltage(), 3);
  Serial.print("\t");
  Serial.print("Batt_Percent:");
  Serial.print(lc.cellPercent(), 1);
  Serial.print("\t");
  Serial.print("Batt_Temp:");
  Serial.println(lc.getCellTemperature(), 1);
  #endif 


// MQTT PUBLISH ==============================
  DynamicJsonDocument doc(1024);
  char buffer[256];

  doc["humidity"] = humidity.relative_humidity;
  doc["temperature"] = temp.temperature;
  doc["battery"] = lc.cellPercent();
  size_t n = serializeJson(doc, buffer);

  bool published = client.publish(stateTopic.c_str(), buffer, n);
  if (published) 
  { 
    printme("Data Sent"); 
  }
  else
  {
    printme("Failed Sending Data"); 
    pixels.fill(0xFF0000);
    pixels.show();
    delay(1000);
  }  
  client.disconnect();
  LEDoff();
  WiFi.disconnect();  
  delay(100);
  disableInternalPower();
  delay(100);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  printme("Going to sleep now");
  esp_deep_sleep_start();

}

void LEDon() {
#if defined(PIN_NEOPIXEL)
  pixels.begin(); // INITIALIZE NeoPixel
  pixels.setBrightness(20); // not so bright
  pixels.setPixelColor(0, 0xFFFFFF);
  pixels.show();
#endif
}

void LEDoff() {
#if defined(PIN_NEOPIXEL)
  pixels.setPixelColor(0, 0x0);
  pixels.show();
#endif
}

void enableInternalPower() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // turn on the I2C power by setting pin to opposite of 'rest state'
  pinMode(PIN_I2C_POWER, INPUT);
  delay(1);
  bool polarity = digitalRead(PIN_I2C_POWER);
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, !polarity);
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif
}

void disableInternalPower() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
#endif

#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, LOW);
#endif

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
  // turn on the I2C power by setting pin to rest state (off)
  pinMode(PIN_I2C_POWER, INPUT);
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
#endif
}


void loop() {
}
