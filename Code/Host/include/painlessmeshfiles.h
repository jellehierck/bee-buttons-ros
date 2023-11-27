#pragma once 



void receivedCallback(const uint32_t &from, const String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionsCallback(uint32_t nodeName);
void nodeTimeAdjustedCallback(int32_t offset);
void delayReceivedCallback(uint32_t from, String msg);

void painlessmesh_setup();
void mesh_loop();

void send_message_host(String message);



