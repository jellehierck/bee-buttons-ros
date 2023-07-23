#pragma once
#include "painlessMesh.h"
#include "SPI.h"
// sd card pins
#define SD_CS 10
#define SPI_MOSI 11 // SD Card
#define SPI_MISO 13
#define SPI_SCK 12

// display pins
#define TFT_WIDTH  135
#define TFT_HEIGHT 240
#define TFT_CS 34
#define TFT_MISO -1
#define TFT_MOSI 35
#define TFT_DC 37
#define TFT_RST 38
#define TFT_SCLK 36
#define TFT_BL 33





struct Config
{
    char Mesh_SSID[64];
    char Mesh_PASSWORD[64];
    int Mesh_PORT;
    int Brightness;
    char Mesh_Name[64];
    int Channel;
};

extern Config config;

extern painlessMesh mesh;


