import serial
import time
import serial
import time

arduino = serial.Serial('com4',9600)

#Allow some time for the serial connection to initialize
time.sleep(2)

while True:
    user_input = input("Enter 'e' or 'r' (Press 'q' to quit): ")

    if user_input.lower() == 'e':
        arduino.write(b'e')
    elif user_input.lower() == 'r':
        arduino.write(b'r')
    elif user_input.lower() == 'q':
        arduino.write(b'q')