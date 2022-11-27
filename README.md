# Bee Button

These are the bee buttons, these buttons can be controlled with the use of MQTT.
They send u if they have been pressed, double pressed or long pressed. And in turn, U can tell them to do light effects or play music. U can use the MQTT protocol with practically every programming language. The only thing u need is to have a MQTT broker like mosquito and be connected to the same network as that MQTT broker and the gateway button. All files related to the project can be found on [Github](https://github.com/utwente-interaction-lab/Bee-Buttons). If u have any questions please mail to m.c.pijnappel@student.utwente.nl

## Starting up

There are two kinds of buttons gateway and normal. This can be changed via the SD Card which will be explained later on.
When starting up they show there state with colors.

### Gateway Button

- Blinking cyan/light blue -> Not connected to anything
- Blinking Blue -> Connected to WiFi
- Blinking Purple -> Connected to MQTT

If after a while it is not blinking purple, there might be something wrong with the WiFi or MQTT broker.
U can start sending commands once it blinks purple

### Normal Button

- Blinking Red -> Not connected to the mesh.
- Blinking Green -> Connected to Mesh.

U can send commands once the gateway is blinking purple and the button u want to target is blinking green.

## Network structure

A MQTT tutorial is available at the Interaction Lab GitHub [here](https://github.com/utwente-interaction-lab/MQTT-Communication)

The network is kinda set up like a beehive. Every group of nodes has one queen/gateway that tells the other buttons what to do and knows what they did. The queen then tells this over Wi-Fi to the MQTT broker. And U can communicate with the queen/gateway as well via the MQTT broker. Besides Wi-Fi, the buttons including the queen/gateway communicate with each other in a mesh network. This means that they do not have to be connected to every node they need to talk to, but can do that via other buttons. So ***u have to make sure that the queen/gateway, your computer and the MQTT broker all are on the same network***, this can be done with the help of the config file on the SD card. Also, Wi-Fi has channels in the case of 2.4Ghz there are 12. U also have to ***make sure that all buttons have this fixed to the same channel***, which can be done in the config file. That channel also has to match one that ur Wi-Fi router/access point uses. U can give this a fixed value in the router software, otherwise, u have to check this with a WiFi scan app and change the config file. ***U can not use eduroam*** for your network, the button cannot connect to this. For the tutorial a router and server are available to use, all buttons configured as a gateway connect to this standard. Do be carefull since all groups connect to this meaning that all groups can send commands to ur buttons.

## SD card

On the SD card, there are a few things.

- Folder "Sounds": here u can store ur **MP3** sound files that u want to play.
- config.txt: Here are some configs u can change related to networking and some default parameters, without having to upload new code.
- README.md: that is what u are reading at the moment.
- Firmware.bin: it is not on there at the moment, but u can update the firmware by putting a firmware.bin file on the SD card.

### config.txt

The config file is meant to quickly change some things without needing to edit the code. Some values are only applicable to the gateway (WIFI_SSID, WIFI_PASSWORD, MQTT_IP, MQTT_PORT). The rest is used by every node.

- **MESH_SSID**: This is the name of the mesh, which has to match among the buttons u use, otherwise, they cannot find each other. U can also see this with ur phone/computer, but u cannot connect to it without a special app.
- **MESH_PASSWORD**: This is the password your network will use between the buttons, so not ur laptop. This also has to match between the buttons.
- **MESH_PORT**: This is the port where they communicate and also has to match. U can have multiple networks match on the MESH_SSID and MESH_PASWORD, but change the port. They won't be able to communicate with each other that way.
- **MESH_NAME**: This is what determines on what topic the messages are published on MQTT.
- **WIFI_CHANNEL**: This is the channel ur wifi uses, but also the buttons. These also have to match among the nodes otherwise they cannot find each other.
- **WIFI_SSID**: This is the name of the Wi-Fi network where your MQTT broker, computer and gateway button are connected to.
- **WIFI_PASSWORD**: This is the password of the Wi-Fi network.
- **MQTT_IP**: This is the IP address of the MQTT broker on the Wi-Fi network.
- **MQTT_PORT**: This is the port where the MQTT protocol talks over, the ***default for mosquitto is 1883***. This can be changed in the broker itself.
- **GATEWAY**: This is the boolean that determines what button will act as the gateway. True means that the button is the gateway, it will still function as a regular button. Only one button can be the gateway in your network.
- **Volume**: This is the volume level the button start with. The scale is 1-21, can be handy to change this on startup. It can also be changed later with MQTT commands, but it won't be remembered between startups.
- **BRIGHTNESS**: The brightness value the LEDs start with, the value is in a range between 0 and 255. Just as with the volume, u can change this with a command but it will not be remembered between reboots.

## Commands

These are all the commands that are available for the buttons with the use of MQTT. The square brackets are not needed and is a variable value. All commands start with a ":" and are also split with a ":". The commands are published on the `MESH_NAME/from/[node id]` and u publish on `MESH_NAME/to/[node id]`. U can send it to every button with `MESH_NAME/to/broadcast`. The gateway publishes and receives with gateway instead of node id.

### Colors

These colors are available to use.

- Red
- Blue
- Green
- Purple
- Yellow
- Orange
- Cyan
- Pink

These are the commands;

- `:Play:[song]` - This is how u play a song on a button. [song] has to be replaced with the name of a song u have put in the sounds folder of the SD card without the ".mp3".
- `:Rainbow` - This will put a rainbow effect on the button.
- `:Full:[color]` - This will put a solid color on the button. Replace [color] with one of the colors.
- `:Circle:[color]` - This will put a circle effect on the button in a color.
- `:Blink:[color]` - This will have the button blink a color.
- `:Clear` - Turns of the LEDs.
- `:Brightness` - With this, u can change the brightness of the button on a scale from 0 - 255.
- `:Volume` - With this u can change the volume of the speaker on a scale from 0-21.
- `:Battery:[subcommand]` - With this, u can get information about the battery.
  - `Percentage` - Gives the battery percentage.
  - `Voltage` - Gives the battery voltage.
  - `Show` - Gives back the battery voltage and percentage, but also show the battery capacity with the LEDs. 

## Extra pins available

When u screw open the button there are some pins extra available to use. U do have to edit the code to use them. The code is written with platformIO and the code is on the GitHub. Installing [platformio](https://platformio.org/) is fairly easy with Visual Studio Code.

- IC3/IC4: These are pins for the I2C protocol, lots of digital sensors can be connected to these pins like accelerometer/gyroscope. 
- Analoge Pins: If u want to do something analoge it can be done with these pins. Make sure it is all 3.3V, since that is what the controller has, has maximum input and supplies.
- LED Strip PINS: If u want to use more LEDs u can connect a neopixel/ws2812b strip on these pins. At the moment this might not work with every buttons since some LEDs are broken and might not pass on the signal to the end. The LED strip would be LED number 13. But in the case of some buttons this might be off since some jumper wires are soldered over LEDs.
