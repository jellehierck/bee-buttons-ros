#include "display.h"
#include "general.h"

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

void displaySetup()
{

    tft.init(TFT_WIDTH, TFT_HEIGHT);
    tft.setRotation(1);
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    tft.fillScreen(ST77XX_RED);
    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);

    tft.print("Name: ");
    tft.println(config.Mesh_SSID);
    tft.setTextSize(1);
    tft.print("Password: ");
    tft.println(config.Mesh_PASSWORD);
    tft.setTextSize(2);
    tft.print("Channel: ");
    tft.println(config.Channel);
    

}

void DisplayNode(int nodeCount){
    tft.setCursor(0, 40);
    tft.print("Nodes: ");
    tft.println(nodeCount);

}

