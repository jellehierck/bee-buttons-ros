#pragma once
#include "painlessMesh.h"
#include "SPI.h"
#include "USB.h"
#include "USBMSC.h"
#include <Preferences.h>


#ifdef TDONGLE_S3

// sd card pins
#define SD_CS 18
#define SPI_MOSI 16 // SD Card
#define SPI_MISO 14
#define SPI_SCK 12

// display pins
#define TFT_WIDTH  80
#define TFT_HEIGHT 160
#define TFT_CS 4
#define TFT_MISO -1
#define TFT_MOSI 3
#define TFT_DC 2
#define TFT_RST 1
#define TFT_SCLK 5
#define TFT_BL 38

// User button
#define BUTTON 0
extern bool displayOrientation;


#endif

struct Config
{
    char Mesh_SSID[64];
    char Mesh_PASSWORD[64];
    int Mesh_PORT;
    // char Mesh_Name[64];
    int Channel;
};

extern Config config;
extern int nodeCount;
extern painlessMesh mesh;
extern Preferences preferences;
#define HWSerial Serial0 
