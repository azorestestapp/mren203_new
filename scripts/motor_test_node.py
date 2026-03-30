#!/usr/bin/env python3
import time
import serial

PORT = "/dev/ttyACM0"
BAUD = 115200

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2.0)

    print("Opened serial port")
    print("Sending 150,150")

    start = time.time()
    while time.time() - start < 3.0:
        ser.write(b"150,150\n")
        time.sleep(0.1)

    print("Sending 0,0")
    for _ in range(10):
        ser.write(b"0,0\n")
        time.sleep(0.1)

    ser.close()
    print("Closed serial port")

if __name__ == "__main__":
    main()