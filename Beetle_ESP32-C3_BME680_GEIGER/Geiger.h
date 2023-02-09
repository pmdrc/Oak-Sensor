/*!
  @file geiger.ino
  @brief    Detect CPM radiation intensity, the readings may have a large deviation at first, and the data tends to be stable after 3 times
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @licence     The MIT License (MIT)
  @author [fengli](li.feng@dfrobot.com)
  @version  V1.0
  @date  2021-9-17
  @get from https://www.dfrobot.com
  @https://github.com/DFRobot/DFRobot_Geiger
*/

#include <DFRobot_Geiger.h>
//#if defined ESP32
//#define detect_pin D3
//#else
#define detect_pin 0
//#endif
/*!
   @brief Constructor
   @param pin   External interrupt pin
*/
DFRobot_Geiger  geiger(detect_pin);

int led = 10;
void GeigerSetup()
{
  //Start counting, enable external interrupt
  geiger.start();
  pinMode(led,OUTPUT);

}

void GeigerMeasure() {
  //Get the current CPM, if it has been paused, the CPM is the last value before the pause
  //Predict CPM by falling edge pulse within 3 seconds, the error is ±3CPM
  digitalWrite(led,HIGH);

  Serial.print(geiger.getCPM());
  Serial.print(" CPM - ");
  //Get the current nSv/h, if it has been paused, nSv/h is the last value before the pause
  Serial.print(geiger.getnSvh());
  Serial.print(" nSvh - ");
  //Get the current μSv/h, if it has been paused, the μSv/h is the last value before the pause
  Serial.print(geiger.getuSvh());
  Serial.println(" uSvh");

  delay(500);
  digitalWrite(led,LOW);
}