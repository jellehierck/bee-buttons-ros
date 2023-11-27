#pragma once

#include "general.h"
#include <Adafruit_GFX.h>  

#ifdef TDONGLE_S2

#include <Adafruit_ST7789.h>

#endif

#ifdef TDONGLE_S3

#include <Adafruit_ST7735.h>

#endif

void displaySetup();
void DisplayNode(int nodeCount);