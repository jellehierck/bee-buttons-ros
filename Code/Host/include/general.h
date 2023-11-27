#pragma once
#include "painlessMesh.h"
#include "SPI.h"
#include "USB.h"
#include "USBMSC.h"
// #ifdef TDONGLE_S2

// // sd card pins
// #define SD_CS 10
// #define SPI_MOSI 11 // SD Card
// #define SPI_MISO 13
// #define SPI_SCK 12

// // display pins
// #define TFT_WIDTH  135
// #define TFT_HEIGHT 240
// #define TFT_CS 34
// #define TFT_MISO -1
// #define TFT_MOSI 35
// #define TFT_DC 37
// #define TFT_RST 38
// #define TFT_SCLK 36
// #define TFT_BL 33
// #endif

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

#endif

struct Config
{
    char Mesh_SSID[64];
    char Mesh_PASSWORD[64];
    int Mesh_PORT;
    char Mesh_Name[64];
    int Channel;
};

extern Config config;

extern painlessMesh mesh;

#define HWSerial Serial0 
