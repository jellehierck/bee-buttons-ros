#include "display.h"
#include "general.h"

#ifdef TDONGLE_S2

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

#endif

#ifdef TDONGLE_S3

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

#endif



void displaySetup()
{

    #ifdef TDONGLE_S2
        tft.init(TFT_WIDTH, TFT_HEIGHT);
        pinMode(TFT_BL, OUTPUT);
        digitalWrite(TFT_BL, HIGH);

        tft.setRotation(1);



    #endif

    #ifdef TDONGLE_S3
        tft.initR(INITR_MINI160x80_PLUGIN);
        pinMode(TFT_BL, OUTPUT);
        digitalWrite(TFT_BL, LOW);

        tft.setRotation(1);

       
    #endif
        tft.fillScreen(ST77XX_RED);
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


    

}

void DisplayNode(int nodeCount){

    tft.fillScreen(ST77XX_RED);
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
    tft.setCursor(0, 40);
    tft.print("Nodes: ");
    tft.println(nodeCount);

}

