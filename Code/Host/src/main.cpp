
// Main file of the code
// dont put extra delays anywhere, this will break the networking part of the buttons

#include <Arduino.h>
#include "general.h"
#include "painlessmeshfiles.h"
#include "display.h"
#include "sdCard.h"
#include <ArduinoJson.h>
// #include "OneButton.h"
#include "button.h"

Config config;
int nodeCount = 0;
// #include "USB.h"

// void testdrawtext(char *text, uint16_t color);
// setup of all functions and button parts are called here
void setup()
{

  Serial.begin(115200);
  button_setup();
  setCpuFrequencyMhz(240);
  delay(2000);


  SDCARD_SETUP();


  delay(1000);
  Update_Firmware();
  displaySetup();

  painlessmesh_setup();
  Serial.println("This is the gateway");
  pinMode(0, INPUT);



}

// all functions that need be kept running are put here
void loop()
{
  button_loop();
  mesh_loop();

// Checks if there was a message on serial
  if (Serial.available())
  {
    String message = Serial.readStringUntil('`');

    if (message != "")
    {
      message.trim();
      
// parses the message received over Serial
      int nodeNumberindexEnd = message.indexOf(':');
      String nodeNumber = message.substring(0, nodeNumberindexEnd);
      String Command = message.substring(nodeNumberindexEnd);

// checks if the message has to go to every node on the network, a specific button or if the nodeList has to be send back
      if (nodeNumber == "Broadcast")
      {
        mesh.sendBroadcast(Command);
      }
      else if (nodeNumber == "Nodelist")
      {
        auto nodes = mesh.getNodeList(true);
        String str = "nodeList";
        nodeCount = -1;
        for (auto &&id : nodes)
        {
          str += String("/") + String(id);
          nodeCount = nodeCount + 1;
        }

        Serial.println(str);
        DisplayNode(nodeCount);
      }
      else
      {

        uint32_t nodeNumberInt = (nodeNumber.substring(0, 9).toInt() * 10) + nodeNumber.substring(9).toInt();
        mesh.sendSingle(nodeNumberInt, Command);
      }
    }
  }
}
