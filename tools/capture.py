#!/usr/bin/env python3
"""
Capture ESP32 serial data to a CSV file.

Usage:
    python3 tools/capture.py                        # defaults
    python3 tools/capture.py --output data/ident.csv
    python3 tools/capture.py --port /dev/ttyACM0 --output data/pid.csv
"""
import serial
import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description='ESP32 serial capture')
    parser.add_argument('--port',   default='/dev/ttyUSB0', help='Serial port')
    parser.add_argument('--baud',   type=int, default=115200, help='Baud rate')
    parser.add_argument('--output', default='capture.csv',   help='Output file')
    parser.add_argument('--end',    default='#',
                        help='End-of-capture line prefix (default: #)')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=None)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        sys.exit(1)

    print(f"Port {args.port} @ {args.baud} baud → {args.output}")
    print("Start the robot (flash or reset). Ctrl+C to cancel.\n")

    with ser, open(args.output, 'w') as f:
        try:
            for raw in ser:
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                # Useful lines: CSV header, numeric data, IMU/ODOM/MOTOR_CURVE frames, end line
                if line.startswith(('t,', 'IMU', 'ODOM_CALIB', 'MOTOR_CURVE', args.end)) or line[0].isdigit():
                    print(line)
                    f.write(line + '\n')
                    f.flush()
                if line.startswith(args.end):
                    break
        except KeyboardInterrupt:
            print('\nManual stop.')

    print(f'\nCapture complete → {args.output}')


if __name__ == '__main__':
    main()
