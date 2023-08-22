
// Main file of the code
// dont put extra delays anywhere, this will break the networking part of the buttons

#include <Arduino.h>
#include "general.h"
#include "painlessmeshfiles.h"
#include "display.h"
#include "sdCard.h"
#include <ArduinoJson.h>
Config config;

// #include "USB.h"


void testdrawtext(char *text, uint16_t color);
// setup of all functions and button parts are called here
void setup()
{

  Serial.begin(115200);

  delay(2000);
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, LOW);

  SDCARD_SETUP();

  // leds_setup();
  delay(1000);
  Update_Firmware();
  displaySetup();

  painlessmesh_setup();
  Serial.println("This is the gateway");
}

// all functions that need be kept running are put here
void loop()
{

  mesh_loop();
  String message = Serial.readStringUntil('`');
  // Serial.readBytes

  if (message != "")
  {
    message.trim();
    // message  += " test";
    int nodeNumberindexEnd = message.indexOf(':');
    String nodeNumber = message.substring(0, nodeNumberindexEnd);
    String Command = message.substring(nodeNumberindexEnd);

    if (nodeNumber == "Broadcast")
    {
      mesh.sendBroadcast(Command);

      // Still have to upload code
      // serial.print(Command)
      // serial.println(" has been send")
    }
    else if(nodeNumber == "Nodelist")
    {
      auto nodes = mesh.getNodeList(true);
            String str = "nodeList";
            int nodeCount = -1;
            for (auto &&id : nodes){
                str += String("/") + String(id);
                nodeCount =nodeCount+ 1;
                }

    Serial.println(str);


    }
    else
    {

      uint32_t nodeNumberInt = (nodeNumber.substring(0, 9).toInt() * 10) + nodeNumber.substring(9).toInt();
      mesh.sendSingle(nodeNumberInt, Command);

      // Still needs to be uploaded
      // serial.print(Command)
      // serial.print(" has been send to")
      // serial.println(nodeNumberInt)


    }
  }
}
