#pragma once
#include "painlessMesh.h"
// sd card pins
#define SD_CS 10
#define SPI_MOSI 11 // SD Card
#define SPI_MISO 13
#define SPI_SCK 12


// i2s pins
#define I2S_DOUT 25
#define I2S_BCLK 27 // I2S
#define I2S_LRC 26

// button pins
#define BUTTON 4

// LED pins
#define DATA_PIN 13

#define BAT_PIN A2 
// #define minBat = 3.3;
// #define maxBat = 4.2;

struct Config
{
    char Mesh_SSID[64];
    char Mesh_PASSWORD[64];
    int Mesh_PORT;
    char WIFI_SSID[64];
    char WIFI_PASSWORD[64];
    char MQTT_IP[14];
    int MQTT_PORT;
    bool GATEWAY_BOOL;
    int Channel;
    int Brightness;
    int Volume;
    char Mesh_Name[64];
};

extern Config config;

extern painlessMesh mesh;


