#include <WiFi.h>
WiFiClient wifiClient;

#include "mqtt.h"
#include "sensors.h"
#include "secrets.h"

#include "Adafruit_NeoPixel.h"
// #include "Adafruit_LC709203F.h"

#define NUMPIXELS      1
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */
#define TIME_SLEEP_LOW 3600        /* Time ESP32 will go to sleep (in seconds) */
#define WAIT_FOR_WIFI  10
#define WAIT_FOR_MQTT  15
//#define DEBUGME        1

#define PIN_NEOPIXEL   5


// How many internal neopixels do we have? some boards have more than one!
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Variables defined in secrets.h
// char ssid[]     = "MySSID";
// char password[] = "WiFiPass";
// const char* mqttServer = "192.168.10.3"; // The IP of your MQTT broker
// const int mqttPort = 1883;
// const char* mqttUser = "homeassistant";
// const char* mqttPassword = "Secret";


void printme(const char* text) {
#if defined(DEBUGME) 
  Serial.print(text);
#endif
}

void setup() 
  {
  bool status;
  char temp_value[50];

  uint32_t BatteryVoltage;      //battery voltage in V

  //visual feedback when we are active, turn on onboard LED
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);


  // Turn on any internal power switches for TFT, NeoPixels, I2C, etc!
  enableInternalPower();
  LEDon();
  delay(50);
  #if defined(DEBUGME) 
  Serial.begin(115200);
  while (!Serial) {
    delay(100); // wait for serial port to connect. Needed for native USB port only
  }
  printme("\nStarting...\n");
  #endif

  //read battery voltage
  BatteryVoltage = readBattery();
  Serial.printf("Voltage: %4.3f V\r\n", BatteryVoltage/1000.0);

  //a reset is required to wakeup again from below CRITICALLY_LOW_BATTERY_VOLTAGE
  //this is to prevent damaging the empty battery by saving as much power as possible
  if (BatteryVoltage < CRITICALLY_LOW_BATTERY_VOLTAGE) {
    Serial.println("Battery critically low, hibernating...");

    //switch off everything that might consume power
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_CPU, ESP_PD_OPTION_OFF);

    //disable all wakeup sources
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  
    digitalWrite(2, LOW);
    LEDoff();
    disableInternalPower();
    esp_deep_sleep_start();

    Serial.println("This should never get printed");
    return;
  }

  //if battery is below LOW_BATTERY_VOLTAGE but still above CRITICALLY_LOW_BATTERY_VOLTAGE, 
  //stop doing the regular work
  //when put on charge the device will wakeup after a while and recognize voltage is OK
  //this way the battery can run low, put still wakeup without physical interaction
  if (BatteryVoltage < LOW_BATTERY_VOLTAGE) {
    Serial.println("Battery low, deep sleeping...");

    //sleep ~60 minutes if battery is CRITICALLY_LOW_BATTERY_VOLTAGE to VERY_LOW_BATTERY_VOLTAGE
    //sleep ~10 minutes if battery is VERY_LOW_BATTERY_VOLTAGE to LOW_BATTERY_VOLTAGE
    uint64_t sleeptime = (BatteryVoltage >= VERY_LOW_BATTERY_VOLTAGE) ? \
                           10*60*1000000ULL : 60*60*1000000ULL;
    
    esp_sleep_enable_timer_wakeup(sleeptime);
    digitalWrite(2, LOW);
    LEDoff();
    disableInternalPower();
    esp_deep_sleep_start();
    
    Serial.println("This should never get printed");
    return;
  }

/*
  if (!lc.begin()) {
    printme("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!\n");
  }
  else {
    lc.setPackSize(LC709203F_APA_1000MAH);
    //lc.setAlarmVoltage(3.8);
  }
  delay(10);
  if (lc.cellPercent() < 5.0)
  {
    Serial.print("Very Low Batt_Percent:");
    Serial.print(lc.cellPercent(), 1);
    Serial.print("\n");
    esp_sleep_enable_timer_wakeup(TIME_SLEEP_LOW * 3 * uS_TO_S_FACTOR);
    printme("Hybernating 3h");
    esp_deep_sleep_start(); 
  }
*/
  if ((status=aht.begin())==false) {
    printme("Could not find AHT? Check wiring\n");
    delay(100);
 //   disableInternalPower();
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    printme("Going to sleep now");
    digitalWrite(2, LOW);
    LEDoff();
    disableInternalPower();
    esp_deep_sleep_start(); 
  }
  else {
    printme("AHT10 or AHT20 found\n");
  }

  // WIFI ============================== We start by connecting to a WiFi network
  pixels.fill(0xFF00FF); // LED MAGENTA
  pixels.show();
  WiFi.setHostname("ESP32-ROOM-2"); 
  WiFi.mode(WIFI_STA);
  printme("Connecting to ");
  printme(ssid);
  printme("\n");
  WiFi.begin(ssid, password);
  if ( WiFi.waitForConnectResult() == WL_CONNECTED )
  {
    #if defined(DEBUGME)
    printme("\n");
    printme("WiFi connected\n");
    printme("IP address: ");
    Serial.println(WiFi.localIP());
    #endif
  }
  else
  {
 //   disableInternalPower();
    delay(100);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    printme("WIFI FAILED Going to sleep now");
    digitalWrite(2, LOW);
    LEDoff();
    disableInternalPower();
    esp_deep_sleep_start(); 
  }

  // MQTT =================================================================
  pixels.fill(0x0000FF); // set LED to blue
  pixels.show();
  client.setServer(mqttServer, mqttPort);
  printme("Connecting to MQTT...\n");
  uint32_t t2 = millis();
  bool mqtt_timeout=false;
  while (!client.connected())
  {
    printme(".");
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
      printme("\n");
    }
    if (millis() - t2 > WAIT_FOR_MQTT * 1000)
    {
      printme("Timeout connecting to MQTT.\n");
      mqtt_timeout = true;
      break;
    }
    delay(100);
  }
  if (mqtt_timeout)
  {
 //   disableInternalPower();
    delay(100);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    printme("Going to sleep now");
    digitalWrite(2, LOW);
    LEDoff();
    disableInternalPower();
    esp_deep_sleep_start(); 
  }
  
// READ SENSORs ============================================== 
  sensors_event_t humidity, temp;

  pixels.fill(0x00FF00); // set LED to GREEN
  pixels.show();
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  #if defined(DEBUGME) 
  Serial.print("Temperature: "); 
  Serial.print(temp.temperature);
  Serial.println(" degrees C");
  Serial.print("Humidity: "); 
  Serial.print(humidity.relative_humidity); 
  Serial.println("% rH");
/*
  Serial.print("Batt_Voltage:");
  Serial.print(lc.cellVoltage(), 3);
  Serial.print("\t");
  Serial.print("Batt_Percent:");
  Serial.print(lc.cellPercent(), 1);
  Serial.print("\t");
  Serial.print("Batt_Temp:");
  Serial.println(lc.getCellTemperature(), 1);
*/ 
  #endif 


// MQTT PUBLISH ==============================
  DynamicJsonDocument doc(1024);
  char buffer[256];

  snprintf(temp_value,sizeof(temp_value),"%0.0f",humidity.relative_humidity);
  doc["humidity"] = temp_value;
  snprintf(temp_value,sizeof(temp_value),"%0.1f",temp.temperature);
  doc["temperature"] = temp_value;
  snprintf(temp_value,sizeof(temp_value),"%3u",calc_battery_percentage(BatteryVoltage)); //lc.cellPercent());
  doc["battery"] = temp_value;
  size_t n = serializeJson(doc, buffer);

  bool published = client.publish(stateTopic.c_str(), buffer, n);
  if (published) 
  { 
    pixels.fill(0xFFFFFF); // set LED to white
    pixels.show();
    delay(100);
    printme("Data Sent\n"); 
  }
  else
  {
    printme("Failed Sending Data\n"); 
    pixels.fill(0xFF0000);
    pixels.show();
    delay(1000);
  }  
  client.disconnect();
  LEDoff();
  WiFi.disconnect();  
  delay(100);
//  disableInternalPower();
//  delay(100);
/*
  if (lc.cellPercent() < 10.0)
  {
    esp_sleep_enable_timer_wakeup(TIME_SLEEP_LOW * uS_TO_S_FACTOR);
    printme("Very Low Battery - Hybernating 24h");
    esp_deep_sleep_start(); 
  }  
*/
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  printme("Going to sleep now\n");
  digitalWrite(2, LOW);
  LEDoff();  
  disableInternalPower();
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
