import serial
import numpy
serial_arduino = serial.Serial('COM3', 115200, timeout=.5)

while True:
    if serial_arduino.inWaiting() > 0:
        a = str(serial_arduino.readline())
        try:
            c = numpy.array([float(i) for i in a.split(' ')[1:6]]) * 0.001536
            # print("\r"+str(c[0])+" "+str(c[1])+" "+str(c[2])+" "+str(c[3])+" "+str(c[4]), end=' ')
            print("\r {}".format(c), end=' ')
        except:
            continue