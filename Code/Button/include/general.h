#pragma once
#include "painlessMesh.h"
// sd card pins
#define SD_CS 5
#define SPI_MOSI 23 // SD Card
#define SPI_MISO 19
#define SPI_SCK 18


// i2s pins
#define I2S_DOUT 25
#define I2S_BCLK 27 // I2S
#define I2S_LRC 26

// button pins
#define BUTTON 4

// LED pins
#define DATA_PIN 13

#define BAT_PIN A2 


struct Config
{
    char Mesh_SSID[64];
    char Mesh_PASSWORD[64];
    int Mesh_PORT;
    int Channel;
    int Brightness;
    int Volume;

};

extern Config config;

extern painlessMesh mesh;


