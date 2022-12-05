// based on https://github.com/256dpi/processing-mqtt


//import mqtt library, make sure u have downloaded the library.
// Since it uses internet it could be windows defender deletes files. Make sure this does not happen since it will break proccesing.
import mqtt.*;

MQTTClient client;

// String array where all node ID are stored
String[] nodeList;
// Mesh name u chose in the config file, u have to change this if u change this
String meshName = "maximilliaan";


////// variables used in the game of this example
int Chosen;
// tracks how many buttons have been pressed correctly
int winCounter = 0;
// How many times u have to press right to win the game
int Win = 3;


void setup() {
  client = new MQTTClient(this);

// if u run proccesing on the laptop that also runs the broke, u can change this if u use a mosquitto on a different ip adderes
  client.connect("mqtt://localhost", "Programm");
  
// if u connect to the computer from the lecture
// client.connect("mqtt://192.168.0.102", "Programm");
  
  // asks for a list of all nodes in the mesh.
  client.publish(meshName + "/to/gateway", ":getNodes");
}

void draw() {
}

// this function is called when proccesing has connected to the MQTT broker
void clientConnected() {
  println("Client has connected");
  // subscribes to all message that the button sends to your mesh
  client.subscribe(meshName + "/from/#");
}


// This function is called when a message arrives on a topic u subscribed to
void messageReceived(String topic, byte[] payload) {

  String Message = new String(payload);
  println("new message: " + topic + " - " + Message);

  //checks where the message comes from
  String subTopic = split(topic, '/')[2];

  // if a message starts with a / it means the list of nodes was sended.
  // This stores all nodes in a list that are connected in the mesh.
  // The id of all nodes that are stored are also on the front of the physical button
  if (Message.charAt(0) == '/') {
    Message = Message.substring(1, Message.length());
    nodeList = split(Message, '/');

    //Changes id of gateway to gateway
    nodeList[0] = "gateway";

    // this is for a game of chase where u have to press the button that is Green
    Chosen =int(random(nodeList.length));

    //turns all buttons blue
    client.publish(meshName + "/to/broadcast", ":Full:Blue");

    //turns chosen button green
    client.publish(meshName + "/to/"+nodeList[Chosen], ":Blink:Green");


    // this checks if the chosen button has been pressed and picks a new one
  }

  if (subTopic.equals(nodeList[Chosen]) == true & Message.equals("Pressed")) {
    println("Chosen has been pressed");
    int oldChosen = Chosen;

    // ensures the same button is not picked again
    while (Chosen == oldChosen) {
      Chosen =int(random(nodeList.length));

      // increases the counter by one when correct button has been pressed
      winCounter++;


      if (winCounter >= Win) {
        client.publish(meshName + "/to/broadcast", ":Rainbow");
        client.publish(meshName + "/to/broadcast", ":Play:dance");
      } else {
        client.publish(meshName + "/to/broadcast", ":Full:Blue");

        client.publish(meshName + "/to/"+nodeList[Chosen], ":Blink:Green");
      }
    }
  }
}

// function that is called when conection to the gateway is lost
void connectionLost() {
  println("connection lost");
}
