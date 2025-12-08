import csv
import sys
from pathlib import Path
from time import sleep
from serial import Serial

RESULTS_DIR = Path(__file__).resolve().parent / "Results"
DEFAULT_PORT = "/dev/cu.mecha16"


def main() -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    log_num = 0
    line = ""
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    try:
        print(f"Opening {port} … (Ctrl-C or 'q'+Enter to quit)")
        with Serial(port, 115200, timeout=0.1) as ser:
            print("Opening Bluetooth connection")
            sleep(0.5)

            print("Flushing Bluetooth buffer")
            while ser.in_waiting:
                ser.read()

            print("Waiting for data...")
            while not ser.in_waiting:
                sleep(0.05)

            while True:
                test_num = input("What test would you like to run? Options are 0-9.\nPress q to Quit.\n")
                if test_num.lower() == "q":
                    break

                ser.write(test_num.encode("utf-8"))
                log_path = RESULTS_DIR / f"log{test_num}.csv"

                with log_path.open("w", newline="") as csv_file:
                    writer = csv.writer(csv_file)
                    header = ser.readline().decode("utf-8", "ignore").strip().split(",")
                    if len(header) != 6:
                        print(f"Unexpected header: {header}")
                        continue
                    writer.writerow(header)

                    while True:
                        if ser.in_waiting:
                            line = ser.readline().decode("utf-8", "ignore").strip()
                            if line == "Test Data Transfer Complete":
                                break

                            parts = line.split(",")
                            if len(parts) != 6:
                                print(f"Invalid data line skipped: {line}")
                                continue

                            try:
                                writer.writerow([float(val) for val in parts])
                            except ValueError:
                                print(f"Non-numeric data skipped: {line}")

                print(f"Test {test_num} data collection complete")
                print(f"CSV saved to {log_path.name}")
                log_num += 1

            ser.write(b"c")
        print("Bluetooth connection closed")
    except KeyboardInterrupt:
        print("Keyboard interrupted")


if __name__ == "__main__":
    main()
    
