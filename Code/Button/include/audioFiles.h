#pragma once 
#include <Arduino.h>
#include "general.h"
#include "Audio.h"




void audio_setup();
void audio_loop();

void play_sound_name(String name);

void Change_Volume(int volume);
void Stop_Music();
