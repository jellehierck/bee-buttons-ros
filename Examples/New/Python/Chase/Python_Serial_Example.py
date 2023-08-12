import random
import serial
import sys
import glob
import serial.tools.list_ports
import time



# Variable that stores the list of nodes
nodeList = []
colorList = ['Red','Blue','Green','Purple','Yellow', 'Orange', 'Cyan', 'Pink']




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
        ser.write(b'Broadcast:Rainbow')

    time.sleep(20)
    
   

while (True):
        # if(ser.read() > 0):
        message = ser.readline().decode('ascii').strip()
        print(message)
        if message.startswith('nodeList/'):
            message = message.replace('nodeList/','')
            nodeList = message.split('/')
            print(nodeList)
            ser.write(b'Broadcast:Blink:Purple')
            print('Game ready')

        elif message.startswith(tuple(nodeList)):
            message = message.split(':')
            node = message[0]
            command = message[1]
            messageOut = node + ':Full:' + str(colorList[random.randint(0,len(colorList)-1)])
            print(messageOut)
            ser.write(messageOut.encode())



