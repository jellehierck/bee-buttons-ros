#pragma once 
#include <Arduino.h>
#include "painlessMesh.h"
#include "general.h"
#include "painlessmeshfiles.h"
#include <PubSubClient.h>
#include <WiFiClient.h>




void Gateway_Mesh_Setup();
void Gateway_Mesh_Loop();

void receivedCallbackA(const uint32_t &from, const String &msg);
void mqttCallback(char *topic, byte *payload, unsigned int length);
void changedConnectionsCallback(uint32_t nodeName);
void send_MQTT_message(String msg);