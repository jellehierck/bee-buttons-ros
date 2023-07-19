// all functions related to interacting with the button (button pressing and commands)

// all files and library needed for using commands and registering button presses
#include <Arduino.h>

#include "painlessmeshfiles.h"
#include "painlessmeshGateway.h"
#include "OneButton.h"
#include "button.h"
#include "general.h"
// #include "lights.h"
// #include "audioFiles.h"

// makes a onebutton object
OneButton *button;

// setup needed for the button, initializes de types of presses and the button itself

void button_setup()
{
    // attach button to the button object (general.h has the button pin declared)

    pinMode(BUTTON, INPUT_PULLUP);
    button = new OneButton();
    button->attachClick([]()
                        { button_pressed(); });

    button->attachDoubleClick([]()
                              { button_double_pressed(); });

    button->attachLongPressStop([]()
                                { button_long_press(); });
}


// function that keep listening to button presses in the void loop
void button_loop()
{
    bool buttonIsPressed = (digitalRead(BUTTON) == LOW);
    button->tick(buttonIsPressed);
}


// function that is started when a single short button press is detected
void button_pressed()
{
    String msg = String("Pressed");
    Serial.println(msg);
// send the message to the gateway or to MQTT directly

    send_message_host(msg);

    if (config.GATEWAY_BOOL)
    {
        send_MQTT_message(msg);
    }
}

// function that is started when a double press is detected
void button_double_pressed()
{

    String msg = String("Double Pressed");
    Serial.println(msg);

// send the message to the gateway or to MQTT directly
    send_message_host(msg);

    if (config.GATEWAY_BOOL)
    {
        send_MQTT_message(msg);
    }
}


// function that is started when a long press is detected
void button_long_press()
{

    String msg = String("Long Press");
    Serial.println(msg);

 // send the message to the gateway or to MQTT directly   
    send_message_host(msg);

    if (config.GATEWAY_BOOL)
    {
        send_MQTT_message(msg);
    }
}



