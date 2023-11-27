
// functions related to the LEDs
#include <Arduino.h>
#include <FastLED.h>
#include "lights.h"
#include "general.h"
#include "battery.h"

#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS 12

// initiates the LEDs

CRGB leds[NUM_LEDS];

// some variables related to LED Functions

uint8_t gHue = 0;
unsigned long Start_Time = millis();
unsigned long previousMillis = 0;
const long interval = 1000;
bool blinkBool = false;
CRGB ColorConverter(String colorConvert);
CRGB colorValue = CRGB::Red;
String activeLedMode = "nothing";
int Battery_Percentage = 49;


// setup of the LEDs
void leds_setup()
{

    FastLED.addLeds<WS2812B, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(config.Brightness);
    FastLED.clear();
    FastLED.show();
}

// The loop for all LED functions, some need to be changed every itteration of the loop.
void leds_loop()
{
    // Part that makes the rainbow effect
    if (activeLedMode == "rainbow")
    {
        fill_rainbow(leds, NUM_LEDS, gHue, 7);
        FastLED.show();
        EVERY_N_MILLISECONDS(20) { gHue++; }
    }
    // Part that does the circle animation
    if (activeLedMode == "circle")
    {
        fadeToBlackBy(leds, NUM_LEDS, 20);
        int pos = beatsin16(13, 0, NUM_LEDS - 1);
        leds[pos] += colorValue;
        FastLED.show();
    }
    // Part that makes the LEDs blink
    if (activeLedMode == "blink")
    {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= interval)
        {
            previousMillis = currentMillis;
            if (blinkBool == false)
            {
                blinkBool = true;
                fill_solid(leds, NUM_LEDS, colorValue);
            }
            else if (blinkBool == true)
            {
                blinkBool = false;
                FastLED.clear();
            }
        }

        FastLED.show();
    }
    // Part that show the battery percentage on the LEDs
    if (activeLedMode == "battery" && Start_Time + 1000 <= millis())
    {
        Start_Time = millis();

                

                int Leds_on = map(BatteryPercentage(), 0, 100, 0,12);


                    if(BatteryPercentage() < 25){
                        fill_solid(leds, NUM_LEDS, CRGB::Black);
                        fill_solid(leds,Leds_on, ColorConverter("Red"));
                    }

                    if (BatteryPercentage() >= 25 && BatteryPercentage() < 50)
                    {
                        fill_solid(leds, NUM_LEDS, CRGB::Black);
                        fill_solid(leds,Leds_on, ColorConverter("Orange"));
                    }

                    if (BatteryPercentage() >= 50 && BatteryPercentage() < 75)
                    {
                        fill_solid(leds, NUM_LEDS, CRGB::Black);
                        fill_solid(leds,Leds_on, ColorConverter("Yellow"));
                    }
                    if (BatteryPercentage() >= 75)
                    {
                        fill_solid(leds, NUM_LEDS, CRGB::Black);
                        fill_solid(leds,Leds_on, ColorConverter("Green"));
                    }
                    



                FastLED.show();
    }
    // Part for LED modes that have no animation only have to be set once.
    if (activeLedMode == "nothing")
    {
        
    }
}

// all these functions are for the LED modes. Some only are static and are done her. The modes with animation are done above

void CircleMode(String color)
{
    colorValue = ColorConverter(color);
    Serial.println(String("Circle the color: ") + color);
    Serial.println(activeLedMode);
    activeLedMode = "circle";
    Serial.println(activeLedMode);
}
void Clear()
{
    Serial.println("cleared all leds");
    activeLedMode = "nothing";
    FastLED.clear();
    FastLED.show();
}

void FullMode(String color)
{
    Serial.println(String("Make button: ") + color);
    activeLedMode = "nothing";
    fill_solid(leds, NUM_LEDS, ColorConverter(color));

    FastLED.show();
}

void Rainbow()
{
    Serial.println("Rainbow mode");
    activeLedMode = "rainbow";
}

void BlinkMode(String color)
{
    colorValue = ColorConverter(color);
    Serial.print("blink mode: ");
    Serial.println(color);
    activeLedMode = "blink";
}

void set_brightness(int brightness)
{

    FastLED.setBrightness(brightness);
    FastLED.show();
    Serial.println(brightness);
}

void Show_Bat()
{

    activeLedMode = "battery";

}

// This converts the String color to a CRGB value so it can be used in functions

CRGB ColorConverter(String colorConvert)
{


    if (colorConvert == "Red")
    {

        return CRGB::Red;
    }

    if (colorConvert == "Blue")
    {

        return CRGB::Blue;
    }

    if (colorConvert == "Green")
    {

        return CRGB::Green;
    }

    if (colorConvert == "Purple")
    {

        return CRGB::Purple;
    }

    if (colorConvert == "Yellow")
    {

        return CRGB::Yellow;
    }

    if (colorConvert == "Orange")
    {

        return CRGB::Orange;
    }

    if (colorConvert == "Cyan")
    {

        return CRGB::Cyan;
    }

    if (colorConvert == "Pink")
    {

        return CRGB::Pink;
    }

    Serial.println(colorConvert);
    return CRGB::White;
}
