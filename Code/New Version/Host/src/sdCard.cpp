
// all functions related to the sd card
#include "sdCard.h"


const char *filename = "/config.txt";

// initializes the sdcard

void SDCARD_SETUP()
{
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    if (!SD.begin(SD_CS))
    {
        Serial.println("Error talking to SD card!");
        
    }
// function that reads the config file
    READ_CONFIG(filename, config);
}
// function that updates the firmware when a firmware file is found on the SDcard

void Update_Firmware()
{
// checks if the file is on the SD card
    File firmware = SD.open("/firmware.bin");

// if firmware is found it turns solid green
    if (firmware)
    {


        Update.begin(firmware.size(), U_FLASH);
        Update.writeStream(firmware);

        if (Update.end())
        {
            Serial.println(F("Update finished!"));
        }
        else
        {
            Serial.println(F("Update error!"));
            Serial.println(Update.getError());
        }
        // when the update is done it closes the file, deletes it and restarts the controller
        firmware.close();
        if (SD.remove("/firmware.bin"))
        {
            Serial.println(F("Firmware rename succesfully!"));
        }
        else
        {
            Serial.println(F("Firmware rename error!"));
        }

        delay(2000);

        ESP.restart();
    }
    else{Serial.println("not found");}
    firmware.close();
}


// function that reads the config file and stores all the values
void READ_CONFIG(const char *filename, Config &config)
{
    File file = SD.open(filename);
    if(file){
    StaticJsonDocument<1024> doc;

    DeserializationError error = deserializeJson(doc, file);
    if (error)
    {
        Serial.println("Could not read config file");
    }else
    {
        // stores all the values from the config file
    config.Mesh_PORT = doc["MESH_PORT"];
    strlcpy(config.Mesh_SSID, doc["MESH_SSID"], sizeof(config.Mesh_SSID));
    strlcpy(config.Mesh_PASSWORD, doc["MESH_PASSWORD"], sizeof(config.Mesh_PASSWORD));
    strlcpy(config.Mesh_Name, doc["MESH_NAME"], sizeof(config.Mesh_Name));

    Serial.println(config.Mesh_PORT);
    config.Channel = doc["WIFI_CHANNEL"];
    config.Brightness = doc["BRIGHTNESS"];

    file.close();
    }
    }
    // when no config file is available some default values are used.
    else{
    
    config.Mesh_PORT = 1111;
    config.Channel = 6;
    strlcpy(config.Mesh_SSID, "test", sizeof(config.Mesh_SSID));
    strlcpy(config.Mesh_PASSWORD, "ditisniettekort", sizeof(config.Mesh_PASSWORD));
    strlcpy(config.Mesh_Name, "test", sizeof(config.Mesh_Name));
    config.Brightness = 100;

    }
}