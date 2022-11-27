
// This code is used when a button is also a gateway
#include "painlessmeshGateway.h"
#include "lights.h"
#include "button.h"
#include "sdCard.h"
#include "interactions.h"

// Variables related to the mesh and gateway

#define HOSTNAME "Button_Mesh_Gateway"
String hostname = "Button_Mesh_Gateway";
#define OTA_PART_SIZE 1024
IPAddress getlocalIP();
IPAddress myIP(0, 0, 0, 0);
// IPAddress mqttBroker(192, 168, 1, 45);
String MQTT_Topic;
WiFiClient wifiClient;
// PubSubClient mqttClient(mqttBroker, 1883, mqttCallback, wifiClient);
PubSubClient mqttClient(config.MQTT_IP, 1883, mqttCallback, wifiClient);

// This function sets all stuff related to WiFi, the mesh and MQTT

void Gateway_Mesh_Setup()
{

    WiFi.hostname(hostname.c_str());

    mesh.setDebugMsgTypes(
        ERROR | STARTUP | CONNECTION |
        DEBUG);

    mesh.init(config.Mesh_SSID, config.Mesh_PASSWORD, config.Mesh_PORT, WIFI_AP_STA, config.Channel);
    mesh.setRoot(true);
    mesh.setContainsRoot(true);
    mesh.stationManual(config.WIFI_SSID, config.WIFI_PASSWORD);
    mesh.setHostname(HOSTNAME);
    mesh.onNewConnection(&changedConnectionsCallback);
    mesh.onReceive(&receivedCallbackA);
    MQTT_Topic = String(config.Mesh_Name) + "/from/gateway";
}

// All functions that have to be called every loop for the mesh and MQTT

void Gateway_Mesh_Loop()
{
    mqttClient.loop();
    mesh.update();

    if (myIP != getlocalIP())
    {
        // when connected to the WiFi network it blinks blue and prints it's IP
        myIP = getlocalIP();
        Serial.println("My IP is " + myIP.toString());
        BlinkMode("Blue");
        Serial.println(config.MQTT_IP);

    }
    // checks if connected to MQTT 
    if (!mqttClient.connected())
    {
        if (mqttClient.connect("painlessMeshClient"))
        {
            mqttClient.publish(MQTT_Topic.c_str(), "Ready!");
            String Subscribe_Topic = String(config.Mesh_Name) + "/to/#";
            mqttClient.subscribe(Subscribe_Topic.c_str());
            mesh.sendBroadcast("Hello");
            // when connected to MQTT it sends a message to it and blinks purple
            BlinkMode("Purple");
        }
    }
}
// This function is called when a MQTT message is received on a topic is subscribed to
void mqttCallback(char *topic, uint8_t *payload, unsigned int length)
{
    char *cleanPayload = (char *)malloc(length + 1);
    memcpy(cleanPayload, payload, length);
    cleanPayload[length] = '\0';
    String msg = String(cleanPayload);
    free(cleanPayload);
    int Mesh_Name_Length = String(config.Mesh_Name).length() + 4;
    String targetStr = String(topic).substring(Mesh_Name_Length);
    Serial.println(topic);
    Serial.println(targetStr);
    Serial.print("message: ");
    Serial.println(msg);

    // checks if ut was ment for itself or a other node on the mesh
    if (targetStr == "gateway")
    {
        if (msg == "getNodes")
        {
            auto nodes = mesh.getNodeList(true);
            String str;
            for (auto &&id : nodes)
                str += String(id) + String(" / ");

            mqttClient.publish(MQTT_Topic.c_str(), str.c_str());
        }
        mesh_commands(msg);
    }
    // if it is a broadcast message it reads the command itself but also sends it on the mesh
    else if (targetStr == "broadcast")
    {
        mesh_commands(msg);
        mesh.sendBroadcast(msg);
    }
    else
    // if the message is meant for a node that is not connected on the mesh network it will return a error over MQTT
    {
        uint32_t target = strtoul(targetStr.c_str(), NULL, 10);

        if (mesh.isConnected(target))
        {
            mesh.sendSingle(target, msg);
        }
        else
        {
            mqttClient.publish(MQTT_Topic.c_str(), "Client not connected!" + target);
            Serial.println("Client not connected!" + target);
        }
        // }
    }
}
// This function is called when there is a change in connection
void changedConnectionsCallback(uint32_t nodeId)
{
    Serial.println(String("New node: ") + nodeId);
    mesh.sendSingle(nodeId, "Hello root node here");
    // BlinkMode("Green");
}
// get the IP of the node on the WiFi network
IPAddress getlocalIP()
{
    return IPAddress(mesh.getStationIP());
}

// this function is called when a message is received on the mesh network
void receivedCallbackA(const uint32_t &from, const String &msg)
{

    Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
    String topic = String(config.Mesh_Name) + "/from/" + String(from);
    mqttClient.publish(topic.c_str(), msg.c_str());
}

// function to send a message to the MQTT broker
void send_MQTT_message(String msg)
{

    mqttClient.publish(MQTT_Topic.c_str(), msg.c_str());
}