# based on https://www.emqx.com/en/blog/how-to-use-mqtt-in-python
# install the library with "pip install paho-mqtt" on ur computer


import random

from paho.mqtt import client as mqtt_client


# Variable that stores the list of nodes
nodeList = []

# some variables related to MQTT localhost is when u use your own laptop
# broker = 'localhost'
broker = "192.168.0.102"

port = 1883
topic = "maximilliaan/"


# variables for the game
winCounter = 0
Win = 3
Chosen = 0


# function that is called when connected to MQTT
def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            # once connected will ask for all nodes on the mesh
            client.publish(topic + "to/gateway", ":getNodes")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client("Example_Python")

    client.on_connect = on_connect
    client.connect(broker, port)
    
    return client


# function to subscribe and track those message on MQTT
def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        #some variables that need to stay global
        global nodeList
        global Chosen
        global winCounter
        global Win


        Message = str(msg.payload.decode("utf-8"))
        Topic = msg.topic
        print(f"Received `{Message}` from `{Topic}` topic")

        # Check where the message comes from 
        subTopic = Topic.split('/')[2]
        
          # if a message starts with a / it means the list of nodes was sended.
        # This stores all nodes in a list that are connected in the mesh.
        # The id of all nodes that are stored are also on the front of the physical button
        if Message[0] == '/':
            Message = Message[1:]
            nodeList = Message.split('/')

            # changes id of the gateway to gateway
            nodeList[0] = "gateway"

            # picks a random node to play the chase game
            Chosen = random.randint(0,len(nodeList)-1)

            # turns all buttons pruple
            client.publish(topic + "to/broadcast", ":Full:Purple")

            # turns the button to press to yellow
            client.publish(topic + "to/" + nodeList[Chosen], ":Blink:Yellow")

        # if the right button is pressed it will increase the counter and picks a new one
        if subTopic == nodeList[Chosen] and Message == "Pressed":

            # ensure the same button is not picked again
            oldChosen = Chosen
            while Chosen == oldChosen:
                Chosen = random.randint(0,len(nodeList)-1)

            winCounter += 1


            # if enough buttons are pressed correctly it wil play a nice song
            if Win == winCounter:
                client.publish(topic + "to/broadcast", ":Rainbow")

                client.publish(topic + "to/broadcast", ":Play:dance")

            else:
                client.publish(topic + "to/broadcast", ":Full:Purple")

                client.publish(topic + "to/" + nodeList[Chosen], ":Blink:Yellow")



    # subscribes to all topics the buttons send to 
    client.subscribe(topic + "from/#")
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)

    client.loop_forever()


if __name__ == '__main__':
    run()