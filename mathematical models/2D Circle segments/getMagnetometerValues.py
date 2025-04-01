import serial
import csv

# Change this to match your COM port (Check in Arduino IDE > Tools > Port)
SERIAL_PORT = "COM11"  
BAUD_RATE = 115200
OUTPUT_FILE = "magnetometer_data.csv"

# Maximum number of samples
MAX_SAMPLES = 5000  

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
    finally:
        ser.close()

# Run the function
capture_serial_data()
