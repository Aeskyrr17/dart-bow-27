"""Save the yaw identification stream from USART10 as MATLAB-ready CSV."""

import argparse
import csv

import serial


HEADER = [
    "timestamp_us", "sample_seq", "state", "position_mrad",
    "gyro_yaw_mrad_s", "motor_speed_mrad_s", "torque_command_mNm",
    "torque_feedback_mNm", "frequency_mHz", "imu_status",
    "motor_rx_seq", "imu_sample_seq",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial port, for example COM5")
    parser.add_argument("--output", required=True, help="Destination CSV file")
    args = parser.parse_args()

    rows = 0
    with serial.Serial(args.port, 115200, timeout=1) as port, \
            open(args.output, "x", newline="", encoding="ascii") as output:
        writer = csv.writer(output)
        writer.writerow(HEADER)
        output.flush()
        try:
            while True:
                raw = port.readline()
                if not raw.endswith(b"\n"):
                    continue
                fields = raw.decode("ascii", errors="ignore").strip().split(",")
                if fields == HEADER:
                    continue
                if len(fields) != len(HEADER):
                    continue
                try:
                    values = [int(field) for field in fields]
                except ValueError:
                    continue
                writer.writerow(values)
                rows += 1
                if rows % 100 == 0:
                    output.flush()
        except KeyboardInterrupt:
            output.flush()
            print(f"Saved {rows} rows to {args.output}")


if __name__ == "__main__":
    main()
