#pragma once 
#include <Arduino.h>
#include "general.h"

#define minBat = 3.3;
#define maxBat = 4.2;

float BatteryVoltage();
int BatteryPercentage();
