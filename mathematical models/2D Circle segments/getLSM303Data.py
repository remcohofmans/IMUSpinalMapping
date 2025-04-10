import serial
import csv

SERIAL_PORT = "COM11"  
BAUD_RATE = 115200
OUTPUT_FILE = "LSM303_Mag.csv"

# Maximum number of samples
MAX_SAMPLES = 2500  

def capture_serial_data():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}. Waiting for data...")

        # Open file in write mode (overwrite)
        with open(OUTPUT_FILE, mode="w", newline="") as file:
            writer = csv.writer(file)

            # Wait for header
            while True:
                line = ser.readline().decode("utf-8").strip()
                print(line)  # Print to console for debugging
                if "Timestamp" in line:
                    writer.writerow(line.split(","))  # Write header
                    print("Header received. Logging data...")
                    break

            sample_count = 0

            # Read and store data until MAX_SAMPLES is reached
            while sample_count < MAX_SAMPLES:
                line = ser.readline().decode("utf-8").strip()
                if line:
                    writer.writerow(line.split(","))  # Write data row
                    print(line)  # Print to console
                    sample_count += 1

        print(f"Captured {MAX_SAMPLES} samples. Data saved to {OUTPUT_FILE}")

    except serial.SerialException as e:
        print(f"Error: {e}")

# Run the function
capture_serial_data()