import serial
import time

PORT = "COM3"   # غيري COM3 إلى رقم Arduino عندك
BAUDRATE = 9600

arduino = serial.Serial(PORT, BAUDRATE, timeout=2)

time.sleep(2)

print("Connected to Arduino!")

while True:
    command = input("Enter command (HOME / STOP / A,j1,j2,j3 / exit): ")

    if command.lower() == "exit":
        break

    arduino.write((command + "\n").encode())

    time.sleep(0.2)

    while arduino.in_waiting:
        response = arduino.readline().decode(errors="ignore").strip()
        print("Arduino:", response)

arduino.close()