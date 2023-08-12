import random
import serial
import sys
import glob
import serial.tools.list_ports
import time


# Variable that stores the list of nodes
nodeList = []





# variables for the game
winCounter = 0
Win = 3
Chosen = 0



def serial_ports():

    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


if __name__ == '__main__':
    # print(serial_ports())
    # ser = serial.Serial(serial_ports()[1],115200)
    # functie die lijst van nodes opvraagt
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "USB Serial Device" in p.description:
            COM = p.device

    ser = serial.Serial(COM,115200)
    if ser.isOpen():
        print("command send")
        ser.write(b'Broadcast:Rainbow')

    time.sleep(20)
    
    if ser.isOpen():
        print("command send")
        ser.write(b'Broadcast:Play:Waterloo')
    # ser.write(b'Broadcast:Rainbow')
    # test = ser.in_waiting()
    # print(test)

while (True):
        # if(ser.read() > 0):
        message = ser.readline().decode('ascii').strip()
        print(message)
        if message.startswith('nodeList/'):
            message = message.replace('nodeList/','')
            nodeList = message.split('/')
            print(nodeList)



