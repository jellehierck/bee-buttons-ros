
#include <Arduino.h>

#include "painlessmeshfiles.h"
#include "painlessmeshGateway.h"

#include "general.h"
#include "lights.h"
#include "audioFiles.h"
# include "battery.h"
// all commands that are detected by the mesh network with the function that need  to be run.
void mesh_commands(String msg)
{

// checks if it is a command 
    if (msg.charAt(0) == ':')
    {

// removes the ":" at the start of the string
        msg.remove(0, 1);
// splits the string on all ":"
        int indexLastSplit = msg.lastIndexOf(':');
        Serial.println(msg);
        int lengthMessage = msg.length();


// checks if there is a subcommand to the command
        if (indexLastSplit == -1)
        {
            Serial.println(msg);
            // Command for turning of LEDs
            if (msg == "Clear")
            {
                Clear();
            }
            // Command for having a rainbow effect
            if (msg == "Rainbow")
            {
                Rainbow();
            }
        }
        else
        {
            // Clean received sub string
            String mode = msg;
            mode.remove(indexLastSplit);
            String Setting = msg;
            Setting.remove(0, indexLastSplit + 1);

            // checks of LEDs have to be turned to a solid color
            if (mode == "Full")
            {

                FullMode(Setting);
            }
            // Circle effect for LEDs
            if (mode == "Circle")
            {
                CircleMode(Setting);
            }
            // Blink effect for LEDs
            if (mode == "Blink")
            {
                BlinkMode(Setting);
            }
            // Plays a song/sound
            if (mode == "Play")
            {
                Stop_Music();
                play_sound_name(Setting);
            }
            // changes volume
            if (mode == "Volume")
            {
                Change_Volume(Setting.toInt());
            }
            // changes LED brightness
            if (mode == "Brightness")
            {
                set_brightness(Setting.toInt());
            }
            // commands related to checking the battery
            if (mode == "Battery"){
                // // reads battery and converts to voltage and percentage
                // float Bat_Value = analogRead(BAT_PIN);
                // Bat_Value = map(Bat_Value, 0, 4095, 0 , 3300);
                
                // Bat_Value = Bat_Value * 1.66666666667;
                // int Bat_Percentage = map(Bat_Value, 3300,4020,0,100);

                // Bat_Value = Bat_Value/1000;
               
                // sends battery percentage 
                if (Setting == "Percentage")
                {
                    String msg = "Battery Percentage: ";
                    msg = msg + BatteryPercentage();

                    send_message_host(msg);
                    if (config.GATEWAY_BOOL)
                    {
                        send_MQTT_message(msg);
                    }
                }
                
                // sends voltage of battery
                if (Setting == "Voltage")
                    {
                        String msg = "Battery Voltage: ";
                        msg = msg + BatteryVoltage();

                    send_message_host(msg);
                    if (config.GATEWAY_BOOL)
                    {
                        send_MQTT_message(msg);
                    }
                }
                
                // sends battery percentage and voltage, but also show it with the LEDs
                if (Setting == "Show")
                {
                    Show_Bat();
                    String msg = "Battery Percentage: ";
                    msg = msg + BatteryPercentage();
                    msg = msg + " Battery Voltage: ";
                    msg = msg + BatteryVoltage();
                    msg = msg + " leds on: ";
                    msg = msg + map(BatteryPercentage(), 0, 100, 0,12);


                    send_message_host(msg);
                    if (config.GATEWAY_BOOL)
                    {
                        send_MQTT_message(msg);
                    }
                }
                
            }
        }
    }
}