import time
import board
import busio

# NOTE: The adafruit_gps library is no longer needed for reading raw data,
# but we keep the module for initial configuration commands.
import adafruit_gps
import supervisor  # Use supervisor.runtime.serial_bytes_available for Pico/CircuitPython

# --- 1. GPS UART Setup ---
# Set the pins used to communicate with the GPS module
rx = board.GP8
tx = board.GP9
# Create the UART object for the GPS module itself (typically 9600 baud)
gps_uart = busio.UART(rx, tx, baudrate=9600, timeout=10)

# --- 2. GPS Configuration ---
# Create a temporary GPS instance just to send configuration commands.
gps = adafruit_gps.GPS(gps_uart, debug=False)

# Turn on the basic GGA and RMC info (what you typically want)
# This sends configuration commands to the GPS module.
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
# Set update rate to once a second (1hz)
gps.send_command(b"PMTK220,1000")

# Give the GPS time to process commands
time.sleep(1)

print("--- Starting Raw NMEA Stream ---")
print("Data will be streamed over USB at 115200 baud.")
print("=" * 40)

# --- 3. Main Loop: Read and Print Raw Data ---
while True:
    # Check if there is any data available from the GPS module's UART
    if gps_uart.in_waiting > 0:
        # Read the entire line from the GPS module
        raw_line = gps_uart.readline()

        if raw_line:
            try:
                # Decode the bytes into a string
                nmea_sentence = raw_line.decode("utf-8").strip()

                # Check if it looks like a valid NMEA sentence
                if nmea_sentence.startswith("$G"):
                    # Print the raw NMEA sentence to the USB serial stream.
                    # This is what the PySimpleGUI host program will read.
                    print(nmea_sentence)

            except UnicodeError:
                # Ignore partial or malformed lines that can't be decoded
                pass

    # We remove the 1.0 second print timing logic since we want to print
    # data immediately as it arrives, which is usually every second (1Hz).
    time.sleep(0.01)  # Short delay to prevent pegging the CPU
