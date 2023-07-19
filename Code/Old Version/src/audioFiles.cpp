
// all function related to audio


// import for library that plays sounds
#include "audioFiles.h"

Audio audio;

// all setup that is needed to be able to play audio
void audio_setup()
{
// seting the I2S pins for the DAC
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);

  Serial.print("Set volume: ");
  Serial.println(config.Volume);
  audio.setVolume(config.Volume);   // 0...21

}
// the function that makes sure music is playing
void audio_loop(){
audio.loop();

}


// the function that plays a song from the SD card
void play_sound_name(String name){

  String song = "/Sounds/" + name + ".mp3";
  audio.connecttoFS(SD, song.c_str());

}

// function that enables changing the volume
void Change_Volume(int volume){
  audio.setVolume(volume);
}

// function that enables stopping the music
void Stop_Music(){

  audio.stopSong();
}

