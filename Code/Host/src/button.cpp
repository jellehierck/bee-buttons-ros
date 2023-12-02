// all functions related to interacting with the button (button pressing and commands)

// all files and library needed for using commands and registering button presses
#include <Arduino.h>

#include "painlessmeshfiles.h"
#include "OneButton.h"
#include "button.h"
#include "general.h"
#include "display.h"

bool displayOrientation = false;

Preferences preferences;
OneButton *button;

// setup needed for the button, initializes de types of presses and the button itself

void button_setup()
{
    // attach button to the button object (general.h has the button pin declared)

    pinMode(BUTTON, INPUT);
    button = new OneButton();
    button->attachClick([]()
                        { button_pressed(); });

    button->attachDoubleClick([]()
                              { button_double_pressed(); });

    button->attachLongPressStop([]()
                                { button_long_press(); });

    preferences.begin("start-state", false);
    displayOrientation = preferences.getBool("disp_orien", false);
}

// function that keep listening to button presses in the void loop
void button_loop()
{
    bool buttonIsPressed = (digitalRead(BUTTON) == LOW);
    button->tick(buttonIsPressed);
}

// function that is started when a single short button press is detected = rotates the screen
void button_pressed()
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
    Serial.println("refresh nodelist");
}

// function that is started when a double press is detected - sends an update nodelist over serial
void button_double_pressed()
{
    displayOrientation = !displayOrientation;
    preferences.putBool("disp_orien", displayOrientation);
    Serial.println("Screen rotated");
    DisplayNode(nodeCount);
}

// function that is started when a long press is detected - sets buttons in battery show mode
void button_long_press()
{

    mesh.sendBroadcast(":Battery:Show");
    Serial.println("Show battery of nodes");
}
