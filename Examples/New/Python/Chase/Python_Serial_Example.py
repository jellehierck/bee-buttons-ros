import random
import serial
import serial.tools.list_ports



# End command with ` 

# Variable that stores the list of nodes
nodeList = []
startBool = False




# variables for the game
winCounter = 0
Win = 3
Chosen = 0

def broadcastCommand(Command):
    Command = "Broadcast:" + Command + '`'
    ser.write(Command.encode())

def commandToNode(nodeID, Command):
    msg = nodeID + ":" + Command + '`'
    ser.write(msg.encode())

def refreshNodelist():
    ser.write(b'Nodelist`')


if __name__ == '__main__':

    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "USB Serial Device" in p.description:
            COM = p.device

    ser = serial.Serial(COM,115200)
    if ser.isOpen():
        print("command send")
        # ser.write(b'Broadcast:Full:Purple`')
        # ser.write(b'Nodelist`')
        broadcastCommand("Full:Purple")
        refreshNodelist()


while (True):
        
        if ser.isOpen():
            message = ser.readline().decode('ascii').strip()
            print("mesage from Dongle: " + message)
            if message.startswith('nodeList/'):
                message = message.replace('nodeList/','')
                nodeList = message.split('/')
                print(nodeList)
                # if len(nodeList) > 2:

                #     Chosen = 1
                #     print(Chosen)
                #     # ser.write(b'Broadcast:Blink:Purple`')
                #     broadcastCommand("Blink:Cyan")

                #     # chosenMessage = nodeList[Chosen] + ':Blink:Pink`'
                #     # ser.write(chosenMessage.encode())

                #     commandToNode(nodeList[Chosen], "Blink:Pink")
                #     ser.write(b'Broadcast:Play:nothing`')

                #     ser.write(b'Broadcast:Volume:3`')

            elif message == "This is the gateway":
                print()
            else:
                messageComponents = message.split(':')
                if len(messageComponents) > 1:
                    node = messageComponents[0]
                    state = messageComponents[1]
                    print(node)
                    print(state)

                    if startBool and node == nodeList[Chosen] and state == 'Pressed':
                        oldChosen = Chosen
                        while Chosen == oldChosen:
                            Chosen = random.randint(1,len(nodeList)-1)
                        print(Chosen)
                        winCounter += 1

                        if Win == winCounter:
                            broadcastCommand("Rainbow")
                            broadcastCommand("Play:Survivor")
                            # ser.write(b'Broadcast:Rainbow`')

                            # ser.write(b'Broadcast:Play:Survivor`')


                            print('You won')
                            winCounter = 0

                        else:
                            broadcastCommand("Full:Purple")
                            commandToNode(nodeList[Chosen], "Full:Yellow")
                            # ser.write(b'Broadcast:Full:Purple`')
                            # chosenMessage = nodeList[Chosen] + ':Full:Yellow`'
                            # ser.write(chosenMessage.encode())



                            print('wincounter: ' + str(winCounter))

                    if (startBool == False) and (state == 'Double Pressed'):
                        startBool = True
                        Chosen = random.randint(1,len(nodeList)-1)
                        # ser.write(b'Broadcast:Full:Purple`')
                        broadcastCommand("Full:Purple")
                        commandToNode(nodeList[Chosen], "Full:Yellow")
                        print(nodeList[Chosen])
                        # chosenMessage = nodeList[Chosen] + ':Full:Yellow`'
                        # ser.write(chosenMessage.encode())
                        print("Game started")

                    if (state == "Long Press"):
                        startBool = False
                        refreshNodelist()
                        broadcastCommand("Play:nothing")
                        broadcastCommand("Volume:3")
                        # ser.write(b'Nodelist`')
                        # ser.write(b'Broadcast:Play:nothing`')
                        # ser.write(b'Broadcast:Volume:3`')



                

            



