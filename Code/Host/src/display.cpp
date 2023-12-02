#include "display.h"
#include "general.h"

#ifdef TDONGLE_S3

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

#endif

void displaySetup()
{

#ifdef TDONGLE_S3
    tft.initR(INITR_MINI160x80_PLUGIN);
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, LOW);

#endif
    DisplayNode(nodeCount);
}

void DisplayNode(int nodeCount)
{
    if (!displayOrientation)
        tft.setRotation(3);
    else if (displayOrientation)
    {
        tft.setRotation(1);
    }

    tft.fillScreen(0x6b5d);
    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);

    tft.print("Name: ");
    tft.println(config.Mesh_SSID);

    tft.setTextSize(1);
    tft.print("Password: ");
    tft.println(config.Mesh_PASSWORD);

    tft.setTextSize(2);
    tft.print("Channel: ");
    tft.println(config.Channel);

    tft.print("Temp: ");
    tft.println(temperatureRead());

    tft.print("Nodes: ");
    tft.println(nodeCount);
}
