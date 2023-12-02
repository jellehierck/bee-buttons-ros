// this code is run when the button is only a button and not a gateway.

#include <Arduino.h>
#include "painlessMesh.h"
#include "general.h"
#include "painlessmeshfiles.h"
#include "display.h"



// This function sets all stuff related to WiFi, the mesh and MQTT
painlessMesh mesh;
void painlessmesh_setup()
{

 

    // mesh.setDebugMsgTypes(
    //     ERROR);

    mesh.init(config.Mesh_SSID, config.Mesh_PASSWORD, config.Mesh_PORT, WIFI_AP_STA, config.Channel);
    mesh.setRoot(true);
    mesh.setContainsRoot(true);
    mesh.onNewConnection(&changedConnectionsCallback);
    mesh.onReceive(&receivedCallback);

}

// All functions that have to be called every loop for the mesh and MQTT

void mesh_loop()
{
    
    mesh.update();


    // checks if connected to MQTT 
    
    }

// This function is called when a MQTT message is received on a topic is subscribed to

// This function is called when there is a change in connection
void changedConnectionsCallback(uint32_t nodeId)
{
    // Serial.println(String("newNode:") + nodeId);
    mesh.sendSingle(nodeId, "Hello root node here");
    auto nodes = mesh.getNodeList(true);
            String str = "nodeList";
            nodeCount = -1;
            for (auto &&id : nodes){
                str += String("/") + String(id);
                nodeCount =nodeCount+ 1;
                }
    Serial.println(str);
    DisplayNode(nodeCount);
    // BlinkMode("Green");
}
// get the IP of the node on the WiFi network


// this function is called when a message is received on the mesh network
void receivedCallback(const uint32_t &from, const String &msg)
{

    Serial.printf("%u:%s\n", from, msg.c_str());
   

}

// function to send a message to the MQTT broker