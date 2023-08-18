import random
import serial
import sys
import glob
import serial.tools.list_ports
import time



# Variable that stores the list of nodes
nodeList = []
startBool = False




# variables for the game
winCounter = 0
Win = 3
Chosen = 0


if __name__ == '__main__':

    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "USB Serial Device" in p.description:
            COM = p.device

    ser = serial.Serial(COM,115200)
    if ser.isOpen():
        print("command send")
        ser.write(b'Broadcast:Full:Purple')

    # time.sleep(20)
    
    

        # ser.write(b'Broadcast:Play:Waterloo')
    # ser.write(b'Broadcast:Rainbow')
    # test = ser.in_waiting()
    # print(test)

while (True):
        
        if ser.isOpen():
            message = ser.readline().decode('ascii').strip()
            print(message)
            if message.startswith('nodeList/'):
                message = message.replace('nodeList/','')
                nodeList = message.split('/')
                print(nodeList)
                startBool = True
                Chosen = random.randint(1,len(nodeList)-1)
                print(Chosen)
                ser.write(b'Broadcast:Full:Purple')
                time.sleep(1)
                chosenMessage = nodeList[Chosen] + 'Full:Yellow'
                ser.write(chosenMessage.encode())
                time.sleep(1)

                ser.write(b'broadcast:Play:nothing')
                time.sleep(1)
                ser.write(b'Broadcast:Volume:3')

            elif message == "This is the gateway":
                print()
            else:
                messageComponents = message.split(':')
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
                        ser.write(b'Broadcast:Rainbow')
                        time.sleep(1)
                        ser.write(b'Broadcast:Play:waterloo')
                        time.sleep(1)
                        print('You won')
                        winCounter = 0

                    else:
                        ser.write(b'Broadcast:Full:Purple')
                        time.sleep(1)
                        chosenMessage = nodeList[Chosen] + 'Full:Yellow'
                        ser.write(chosenMessage.encode())
                        time.sleep(1)


                        print('wincounter: ' + str(winCounter))


                

            



