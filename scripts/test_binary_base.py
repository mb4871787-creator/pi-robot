#!/usr/bin/env python3
import serial
import time
import sys
import struct
import argparse

COMMANDS = {
    'FWD': 0x01, 'BWD': 0x02, 'LEFT': 0x03, 'RIGHT': 0x04,
    'CW': 0x05, 'CCW': 0x06, 'FL': 0x07, 'FR': 0x08,
    'BL': 0x09, 'BR': 0x0A, 'STOP': 0x00
}

def send_cmd(port, cmd, speed):
    pkt = struct.pack('BBB', 0xAA, COMMANDS[cmd], speed)
    port.write(pkt)
    print(f"Sent: {cmd} speed={speed}  pkt=[{pkt.hex()}]")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyUSB1')
    parser.add_argument('--auto', action='store_true')
    args = parser.parse_args()

    port = serial.Serial(args.port, 115200, timeout=1)
    time.sleep(2)  # wait for ESP boot

    if args.auto:
        for cmd in ['FWD','BWD','LEFT','RIGHT','CW','CCW','STOP']:
            send_cmd(port, cmd, 120)
            time.sleep(1.0)
    else:
        print("Interactive mode. Commands: FWD BWD LEFT RIGHT CW CCW FL FR BL BR STOP")
        print("Format: CMD SPEED (e.g. 'FWD 120' or 'STOP 0')")
        while True:
            try:
                line = input("> ").strip().upper()
                if not line: continue
                parts = line.split()
                cmd = parts[0]
                speed = int(parts[1]) if len(parts) > 1 else 120
                if cmd in COMMANDS:
                    send_cmd(port, cmd, speed)
                else:
                    print("Unknown command")
            except (EOFError, KeyboardInterrupt):
                break

    send_cmd(port, 'STOP', 0)
    port.close()

if __name__ == '__main__':
    main()
