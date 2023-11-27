import random
import serial
import serial.tools.list_ports



# End command with ` 

# Variable that stores the list of nodes
nodeList = []
startBool = False




# variables for the game
winCounter = 0
Win = 8
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
        print("===========================================================================")
        print("To start the game double press a button.")
        print("To make sure all buttons are connected, long press a button.")
        print("The screen will show how many buttons are connected.")
        print("All purple buttons are connected.")
        print("===========================================================================")

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
                            broadcastCommand("Play:Money")



                            print('You won')
                            print("To reset long press")
                            print("To start new game double press after reset")
                            winCounter = 0

                        else:
                            broadcastCommand("Full:Purple")
                            commandToNode(nodeList[Chosen], "Full:Yellow")




                            print('wincounter: ' + str(winCounter))

                    if (startBool == False) and (state == 'Double Pressed'):
                        startBool = True
                        Chosen = random.randint(1,len(nodeList)-1)
                        broadcastCommand("Full:Purple")
                        commandToNode(nodeList[Chosen], "Full:Yellow")
                        print(nodeList[Chosen])

                        print("Game started")

                    if (state == "Long Press"):
                        startBool = False
                        refreshNodelist()
                        broadcastCommand("Full:Purple")
                        broadcastCommand("Play:nothing")
                        broadcastCommand("Volume:3")




                

            



