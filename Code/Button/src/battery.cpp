#include "battery.h"


// gives the voltage of the battery
float BatteryVoltage()
{

    float Bat_Value = analogRead(BAT_PIN);
    Bat_Value = map(Bat_Value, 0, 4095, 0, 3300);
    Bat_Value = Bat_Value * 1.66666666667;

    return Bat_Value / 1000;
}

// gives the percentage of the battery
int BatteryPercentage()
{

    float Bat_Value = 1000 * BatteryVoltage();

    int Bat_Percentage = map(Bat_Value, 3300, 4020, 0, 100);

    return Bat_Percentage;
}