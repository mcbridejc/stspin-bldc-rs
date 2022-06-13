'''Really simple script to send commands to alternate the speed between two values'''
import serial
import struct
import time

PORT = '/dev/ttyUSB1'

SYNC1 = 2
SYNC2 = 3

def rpm2speed(rpm):
    # 7 electrical revs per mechanical rev
    # 60 RPM per RPS
    # 100 scale factor for units
    return int(rpm * 7 * 100 / 60)

def build_message(id, data):
    if len(data) > 255:
        raise ValueError("Max data length is 255")
    if id > 65535 or id < 0:
        raise ValueError("id must be 16-bit")

    msg = struct.pack("<BBHB", SYNC1, SYNC2, id, len(data))
    msg += data
    crca = 0
    crcb = 0
    msg += struct.pack("<bb", crca, crcb)
    return msg

port = serial.Serial(PORT, baudrate=9600, timeout=1)

steps = [200, 500, 800, 1200]

while True:
    time.sleep(5.0)
    msg = build_message(0, struct.pack("<l", rpm2speed(0)))
    print(f"Sending {msg}")
    port.write(msg)
    for rpm in steps:
        time.sleep(5.0)
        msg = build_message(0, struct.pack("<l", rpm2speed(rpm)))
        print(f"sending {msg}")
        port.write(msg)
