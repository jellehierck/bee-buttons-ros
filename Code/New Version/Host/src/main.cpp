
// Main file of the code
// dont put extra delays anywhere, this will break the networking part of the buttons

#include <Arduino.h>
#include "general.h"
#include "painlessmeshfiles.h"
#include "audiofiles.h"
#include "button.h"
#include "lights.h"
#include "sdCard.h"
#include <ArduinoJson.h>
#include "battery.h"
Config config;
#include "painlessmeshGateway.h"

// setup of all functions and button parts are called here
void setup()
{

  Serial.begin(115200);
  delay(2000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  SDCARD_SETUP();
  leds_setup();
  delay(1000);
  Update_Firmware();

  Serial.println(BatteryPercentage());
  // only lets the code run when the battery is full enough
  if (BatteryPercentage() > 2)
  {
    // checks if a button is a gateway or just a normal button.

    if (config.GATEWAY_BOOL)
    {
      Gateway_Mesh_Setup();
      Serial.println("This is the gateway");
      BlinkMode("Cyan");
    }
    else if (!config.GATEWAY_BOOL)
    {
      painlessmesh_setup();
      Serial.println("this is not the gateway!");
      BlinkMode("Red");
    }

    if (String(config.Mesh_PASSWORD) == "ditisniettekort" & String(config.Mesh_SSID) == "test")
    {
      BlinkMode("Yellow");
    }

    audio_setup();

    button_setup();
  }
  else if (BatteryPercentage() < 3)
  {

    Show_Bat();
  }
}

// all functions that need be kept running are put here
void loop()
{
  leds_loop();
  // only leds the code run when the battery is full enough
  if (BatteryPercentage() > 2)
  {
    audio_loop();

    button_loop();
    // checks if a button is a gateway or just a normal button.
    if (config.GATEWAY_BOOL)
    {
      Gateway_Mesh_Loop();
    }
    else if (!config.GATEWAY_BOOL)
    {
      mesh_loop();
    }
  }
  else if (BatteryPercentage() < 3)
  {

    Show_Bat();
  }
}