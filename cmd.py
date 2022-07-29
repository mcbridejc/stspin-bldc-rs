'''Really simple script to send commands to alternate the speed between two values'''
import click
import serial
import struct

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
    for b in msg:
        crca = (crca + b) % 256
        crcb = (crca + crcb) % 256
    msg += struct.pack("<bb", crca, crcb)
    return msg

@click.command()
@click.option('-p', '--port')
@click.option('--rpm', required=False)
@click.option('--base_pwr', required=False)
@click.option('--accel_pwr', required=False)
@click.option('--accel_rate', required=False)
@click.option('--pwr_per_microrps', required=False)
def main(port, rpm, base_pwr):
    sp = serial.Serial(port, baudrate=9600, timeout=1)

    def send_i32(id, value):
        msg = build_message(id, struct.pack("<l", value))
        sp.write(msg)
    
    cmd_issued = False

    if base_pwr is not None:
        print(f"Setting base power to {int(base_pwr)}")
        send_i32(2, int(base_pwr))
        cmd_issued = True

    if rpm is not None:
        raw_speed = rpm2speed((int(rpm)))
        print(f"Setting RPM to {rpm} (cmdvalue: raw_speed)")
        send_i32(0, rpm2speed(int(rpm)))
        cmd_issued = True


if __name__ == '__main__':
    main()
