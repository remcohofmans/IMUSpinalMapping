import serial
import csv

SERIAL_PORT = "COM11"  # Change as needed
BAUD_RATE = 115200
OUTPUT_FILE = "magnetometer_data_raw_offset_cal.csv"

# Define the expected header
HEADER = [
    "Timestamp", "Unit", "X_raw", "Y_raw", "Z_raw", 
    "X_offset", "Y_offset", "Z_offset", "X_cal", "Y_cal", "Z_cal"
]

# Maximum number of samples
MAX_SAMPLES = 2500  

def capture_serial_data():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}. Logging data...")

        # Open file in write mode (overwrite existing data)
        with open(OUTPUT_FILE, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(HEADER)  # Write the predefined header

            sample_count = 0

            # Read and store data until MAX_SAMPLES is reached
            while sample_count < MAX_SAMPLES:
                line = ser.readline().decode("utf-8").strip()
                if line:
                    data = line.split(",")
                    if len(data) == len(HEADER):  # Ensure correct data format
                        writer.writerow(data)
                        print(line)  # Print to console
                        sample_count += 1
                    else:
                        print(f"Skipping malformed data: {line}")

        print(f"Captured {MAX_SAMPLES} samples. Data saved to {OUTPUT_FILE}")

    except serial.SerialException as e:
        print(f"Error: {e}")
    finally:
        if ser.is_open:
            ser.close()

# Run the function
capture_serial_data()
