#pragma once 
#include <Arduino.h>
#include "general.h"
#include "SD.h"
#include "FS.h"

#include <ArduinoJson.h>
// struct Config
// {
//     char Mesh_SSID[64];
//     char Mesh_PASSWORD[64];
//     int Mesh_PORT;
//     char WIFI_SSID[64];
//     char WIFI_PASSWORD[64];
//     char MQTT_IP[14];
//     int MQTT_PORT;
//     char GATEWAY_BOOL[1];
// };

void SDCARD_SETUP();
void READ_CONFIG(const char *filename, Config &config);
void Update_Firmware();