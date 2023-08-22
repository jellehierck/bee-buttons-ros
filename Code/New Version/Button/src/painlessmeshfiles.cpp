// this code is run when the button is only a button and not a gateway.

#include <Arduino.h>
#include "painlessMesh.h"
#include "general.h"
#include "painlessmeshfiles.h"
#include "lights.h"
#include "audioFiles.h"
// #include "button.h"

#include "interactions.h"
// some variables needed for the mesh network
Scheduler userScheduler;
painlessMesh mesh;

bool calc_delay = false;
SimpleList<uint32_t> nodes;

uint32_t hostNodeIdUint = 0;
// setup for the meshnetwork
void painlessmesh_setup()
{

  mesh.setDebugMsgTypes(ERROR | DEBUG); // set before init() so that you can see error messages
  mesh.setContainsRoot(true);

  mesh.init(config.Mesh_SSID, config.Mesh_PASSWORD, &userScheduler, config.Mesh_PORT, WIFI_AP_STA, config.Channel);
  Serial.println(mesh.getNodeId()); 
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);
  mesh.initOTAReceive("node");



}
// the function with all things related to the meshnetwork that have to be called every loop
void mesh_loop()
{
  mesh.update();
}

// this function is called when a message is received on the mesh network
void receivedCallback(uint32_t from, String &msg)
{

  Serial.printf("Received message by name from: %u, %s\n", from, msg.c_str());
// Sends out a message when it is connected to the gateway and turns the button green
  if (msg == "Hello root node here")
  {
    hostNodeIdUint = from;
    // mesh.sendSingle(hostNodeIdUint, String("Node id: ") + String(mesh.getNodeId()));
    BlinkMode("Green");
  }
  // The function that has all interactions (LEDs and Sounds)
  mesh_commands(msg);

}

// This function is called when a new node is connnected to the mesh network
void newConnectionCallback(uint32_t nodeId)
{
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
  Serial.printf("--> startHere: New Connection, %s\n", mesh.subConnectionJson(true).c_str());

    mesh.sendBroadcast(String("Node id: ") + String(mesh.getNodeId()));
    BlinkMode("Green");

}


// This function is called when there is a change in connection
void changedConnectionCallback()
{
  Serial.printf("Changed connections\n");

  nodes = mesh.getNodeList();

  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("Connection list:");

  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end())
  {
    Serial.printf(" %u", *node);
    node++;
  }
  Serial.println();
  calc_delay = true;
}


// This function is called when the time difference is adjusted
void nodeTimeAdjustedCallback(int32_t offset)
{
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}
// function that calculates the delay between nodes
void delayReceivedCallback(uint32_t from, int32_t delay)
{
  Serial.printf("Delay to node %u is %d us\n", from, delay);
}
// function to send a message to mesh
void send_message_host(String message)
{

  mesh.sendBroadcast(message);
}
