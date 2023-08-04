import random
import serial
import sys
import glob



# Variable that stores the list of nodes
nodeList = []





# variables for the game
winCounter = 0
Win = 3
Chosen = 0



def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
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
    print(serial_ports()[0])
    ser = serial.Serial(serial_ports()[0],115200)
    # functie die lijst van nodes opvraagt
    ser.write(b'Broadcast:blue')
    # test = ser.in_waiting()
    # print(test)

while (True):
        # if(ser.read() > 0):
        message = ser.readline().decode('ascii').strip()
        print(message)


