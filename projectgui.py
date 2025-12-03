import FreeSimpleGUI as sg
import time
import serial, sys
import pynmea2
from threading import Thread, Event
from usbmonitor import USBMonitor
from usbmonitor.attributes import ID_MODEL, ID_MODEL_ID, ID_VENDOR_ID

#Boolean for USB state and GPS data reading state
USB_Found = True

port = "COM3"
baudrate = 9600



    


THREAD_MESSAGE_KEY = '-THREAD-MESSAGE-'
USB_Status = 'No USB Device Found'
# --- Helper function to parse telemetry lines ---
def parse_telemetry_line(line):
    """
    Parse a telemetry line from the device.
    Returns a dict with extracted fields or None if line doesn't match a known format.
    Handles: Latitude, Longitude, Elevation, Acceleration (X/Y/Z), Gyro (X/Y/Z), Magnetic (X/Y/Z).
    """
    import re

    if not line or ':' not in line:
        return None

    parts = line.split(':', 1)
    if len(parts) != 2:
        return None

    key = parts[0].strip().lower()
    val_str = parts[1].strip()

    # Handle multi-axis lines like:
    # "Acceleration: X: -2.22, Y: 0.37, Z: 9.67 m/s^2"
    # "Gyro          X: 0.01, Y: -0.02, Z: 0.01 rad/s"
    if any(k in key for k in ('acceleration', 'gyro', 'magnetic')):
        # Find axis:value pairs within the remainder of the line
        pairs = re.findall(r'([XYZxyz])\s*:\s*([+-]?\d+(?:\.\d+)?)', val_str)
        result = {}
        if pairs:
            for axis, val in pairs:
                axis = axis.upper()
                try:
                    num = float(val)
                except ValueError:
                    continue
                if 'acceleration' in key:
                    result[f'acceleration_{axis.lower()}'] = f"{num:.2f}"
                elif 'gyro' in key:
                    result[f'angular_velocity_{axis.lower()}'] = f"{num:.2f}"
                elif 'magnetic' in key:
                    result[f'magnetic_field_{axis.lower()}'] = f"{num:.2f}"
            # If the first axis (e.g. X) was included in the key (e.g. "Gyro X: 0.01,...")
            # it may not appear in the regex pairs. Try to extract a leading numeric
            # token and assign it to that axis if missing.
            for axis in ('x', 'y', 'z'):
                if axis in key and (
                    (('acceleration_' + axis) not in result) and
                    (('angular_velocity_' + axis) not in result) and
                    (('magnetic_field_' + axis) not in result)
                ):
                    m = re.search(r'([+-]?\d+(?:\.\d+)?)', val_str)
                    if m:
                        try:
                            num = float(m.group(1))
                        except ValueError:
                            continue
                        if 'acceleration' in key:
                            result[f'acceleration_{axis}'] = f"{num:.2f}"
                        elif 'gyro' in key:
                            result[f'angular_velocity_{axis}'] = f"{num:.2f}"
                        elif 'magnetic' in key:
                            result[f'magnetic_field_{axis}'] = f"{num:.2f}"
            return result if result else None
        else:
            # Sometimes the first axis is embedded in the key (e.g. "Gyro    X: 0.01, Y:...")
            # If key contains 'x'/'y'/'z', parse first number as that axis
            for axis in ('x', 'y', 'z'):
                if axis in key:
                    # try to parse the first numeric token in val_str
                    m = re.search(r'([+-]?\d+(?:\.\d+)?)', val_str)
                    if m:
                        try:
                            num = float(m.group(1))
                        except ValueError:
                            return None
                        if 'acceleration' in key:
                            return {f'acceleration_{axis}': f"{num:.2f}"}
                        elif 'gyro' in key:
                            return {f'angular_velocity_{axis}': f"{num:.2f}"}
                        elif 'magnetic' in key:
                            return {f'magnetic_field_{axis}': f"{num:.2f}"}
            return None

    # Fallback: single numeric value lines like "Latitude: 36.124786 degrees"
    tokens = val_str.split()
    if not tokens:
        return None
    try:
        value = float(tokens[0])
    except ValueError:
        return None

    if 'latitude' in key:
        return {'latitude': f"{value:.6f}"}
    elif 'longitude' in key:
        return {'longitude': f"{value:.6f}"}
    elif 'height' in key or 'elevation' in key or 'geoid' in key:
        return {'elevation': f"{value:.2f}"}
    elif 'satellite' in key:
        return {'satellites': str(int(value))}

    return None

# --- Serial Communication and Parsing Thread ---

def serial_reader_thread(window, stop_event: Event):
    """
    Background thread to read data from the Pico via the serial port.
    It sends parsed data or status messages back to the main GUI loop.
    Collects telemetry fields (lat, lon, accel, gyro, mag) until it has a complete set, then sends.
    """
    ser = None
    # Track whether last parsed NMEA indicates a valid GPS fix
    gps_has_fix = False
    # Make sure to update globals we write to
    global USB_Found, running
    accumulated_data = {}  # Accumulates telemetry fields from multiple lines
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        # Send initial status message back to the GUI (thread-safe)
        try:
            window.write_event_value(THREAD_MESSAGE_KEY, {'status': 'Serial Port Opened.'})
        except Exception:
            # If the window doesn't have a thread queue (closed), stop the thread
            USB_Found = False
            running = False
           
            USB_Status = 'No USB Device Found'
            window[USBKEY].update('No USB Device Found')
            window['-STARTSTOP-'].update(disabled=True)
            return
        
        consecutive_errors = 0
        max_consecutive_errors = 3
        
        while True:
            # Check if the main thread requested stop
            if stop_event.is_set():
                break
                
            # Use blocking readline() with timeout; this is more reliable than polling in_waiting
            try:
                raw = ser.readline()
                consecutive_errors = 0  # Reset error counter on successful read
            except (PermissionError, OSError) as e:
                # ClearCommError or device-level errors; try to recover
                consecutive_errors += 1
                if consecutive_errors >= max_consecutive_errors:
                    try:
                        window.write_event_value(THREAD_MESSAGE_KEY,
                                               {'read error': f"Device error after {max_consecutive_errors} attempts. Attempting to reconnect..."})
                    except Exception:
                        pass
                    # Try to close and reopen the port
                    try:
                        if ser and ser.is_open:
                            ser.close()
                        time.sleep(1)
                        ser = serial.Serial(port, baudrate, timeout=0.1)
                        consecutive_errors = 0
                        print("Reconnected to serial port.")
                        try:
                            window.write_event_value(THREAD_MESSAGE_KEY, {'status': 'Reconnected to serial port.'})
                        except Exception:
                            pass
                    except Exception as reconnect_err:
                        print(f"Reconnect failed: {reconnect_err}")
                        USB_Status = 'No USB Device Found'
                        window['-STARTSTOP-'].update(disabled=True)
                        
                        break
                time.sleep(0.1)
                raw = b''
            except Exception as e:
                print(f"Serial read error: {e}")
                consecutive_errors += 1
                if consecutive_errors >= max_consecutive_errors:
                    USB_Found = False
                    running = False
                    break
                time.sleep(0.1)
                raw = b''

            if not raw:
                # No data during timeout period
                time.sleep(0.01)
                continue

            try:
                line = raw.decode('utf-8', errors='replace').strip()
            except Exception as e:
                print(f"Decode error: {e} -- raw: {raw}")
                line = ''



            # Try to parse as telemetry (device format: "Latitude: 36.124786 degrees")
            parsed = parse_telemetry_line(line)
            if parsed:
                accumulated_data.update(parsed)
                # Update status if GPS has a fix (NMEA parsing updates gps_has_fix)
                if gps_has_fix:
                    try:
                        window[USBKEY].update('GPS Fixed, data will now update')
                    except Exception:
                        pass
                accumulated_data['status'] = f"Data Received at {time.strftime('%H:%M:%S')}"
                accumulated_data['raw'] = line
                try:
                    window.write_event_value(THREAD_MESSAGE_KEY, accumulated_data.copy())
                except Exception:
                    break
                    # Clear for next batch
                accumulated_data = {}
            # Also handle standard NMEA sentences (in case device ever sends them)
            elif line and line.startswith('$G'):
                try:
                    msg = pynmea2.parse(line)

                    # Only process RMC or GGA sentences which typically contain position
                    if msg.sentence_type in ['RMC', 'GGA']:
                        # Extract latitude/longitude with fallbacks for attribute names
                        lat = getattr(msg, 'latitude', None)
                        lon = getattr(msg, 'longitude', None)
                        lat_dir = getattr(msg, 'lat_dir', getattr(msg, 'latitude_direction', ''))
                        lon_dir = getattr(msg, 'lon_dir', getattr(msg, 'longitude_direction', ''))

                        lat_str = ''
                        lon_str = ''
                        # Update our internal GPS fix status
                        gps_has_fix = False
                        try:
                            if lat is not None:
                                latf = float(lat)
                                if latf != 0.0:
                                    lat_str = f"{latf:.6f} {lat_dir}"
                                    gps_has_fix = True
                            if lon is not None:
                                lonf = float(lon)
                                lon_str = f"{lonf:.6f} {lon_dir}"
                        except Exception:
                            # If conversion fails, just use raw string values
                            lat_str = str(lat) if lat is not None else ''
                            lon_str = str(lon) if lon is not None else ''

                        if gps_has_fix:
                            data = {
                                'latitude': lat_str,
                                'longitude': lon_str,
                                'status': f"Data Received at {time.strftime('%H:%M:%S')}",
                                'raw': line,
                            }
                            # Send the structured data back to the GUI (thread-safe)
                            try:
                                window.write_event_value(THREAD_MESSAGE_KEY, data)
                            except Exception:
                                break
                        else:
                            try:
                                window.write_event_value(THREAD_MESSAGE_KEY, {'status': 'Waiting for valid GPS Fix...', 'raw': line})
                            except Exception:
                                break
                except pynmea2.ParseError:
                    # Ignore malformed NMEA sentences
                    pass
                except Exception as e:
                    print(f"Parsing error: {e}")

            time.sleep(0.01) # Small delay to prevent pegging the CPU
            
    except serial.SerialException as e:
        # Send error message if the port couldn't be opened or was disconnected
        try:
            window.write_event_value(THREAD_MESSAGE_KEY,
                                     {'status': f"ERROR: Could not open port {port}. ({e})"})
        except Exception:
            # If window not available, just print and exit
            USB_Found = False
            running = False
            USB_Status = 'No USB Device Found'
            window[USBKEY].update(USB_Status)
            window['-STARTSTOP-'].update(disabled=True)            
            print(f"ERROR: Could not open port {port}. ({e})")
            
            
    except Exception as e:
        print(f"Thread Error: {e}")
        
    finally:
        if ser and ser.is_open:
            ser.close()
        USB_Found = False
        running = False
sg.theme('Dark Green 7')

# Define keys for output values
LATKEY = '-LAT-'
LONGKEY = '-LONG-'
ELEVKEY = '-ELEV-'
ANGVELXKEY = '-ANGVELX-'
ANGVELYKEY = '-ANGVELY-'    
ANGVELZKEY = '-ANGVELZ-'
ACCELXKEY = '-ACCELX-'
ACCELYKEY = '-ACCELY-'
ACCELZKEY = '-ACCELZ-'
MFIELDXKEY = '-MFIELDX-'
MFIELDYKEY = '-MFIELDY-'
MFIELDZKEY = '-MFIELDZ-'
USBKEY = '-USBSTATUS-'

# Initialize all output values
latitude_value = '0.0'
longitude_value = '0.0'
elevation_value = '0.0'
angular_velocity_x = '0.0'
angular_velocity_y = '0.0'
angular_velocity_z = '0.0'
acceleration_x = '0.0'
acceleration_y = '0.0'
acceleration_z = '0.0'
magnetic_field_x = '0.0'
magnetic_field_y = '0.0'
magnetic_field_z = '0.0'

running = False
runningText = 'Start'

def reset_values():
    global latitude_value, longitude_value, elevation_value
    global angular_velocity_x, angular_velocity_y, angular_velocity_z
    global acceleration_x, acceleration_y, acceleration_z
    global magnetic_field_x, magnetic_field_y, magnetic_field_z
    latitude_value = '0.0'
    longitude_value = '0.0'
    elevation_value = '0.0'
    angular_velocity_x = '0.0'
    angular_velocity_y = '0.0'
    angular_velocity_z = '0.0'
    acceleration_x = '0.0'
    acceleration_y = '0.0'
    acceleration_z = '0.0'
    magnetic_field_x = '0.0'
    magnetic_field_y = '0.0'
    magnetic_field_z = '0.0'

reset_values()  
starting_time = time.time()
buttonDuo=[sg.Button(runningText, key='-STARTSTOP-',disabled=not USB_Found), sg.Button('Exit'), sg.Text(USB_Status, key=USBKEY)]
layout = [
    [sg.Text('GPS', font=('Any', 16))],
    [sg.Text('Latitude:', size=(20, 1)), sg.Text(latitude_value, key=LATKEY, size=(20, 1))],
    [sg.Text('Longitude:', size=(20, 1)), sg.Text(longitude_value, key=LONGKEY, size=(20, 1))],
    [sg.Text('Elevation:', size=(20, 1)), sg.Text(f'{elevation_value} meters geoid', key=ELEVKEY, size=(20, 1))],
    [sg.Text('Angular Velocity X:', size=(20, 1)), sg.Text(angular_velocity_x, key=ANGVELXKEY, size=(20, 1))],
    [sg.Text('Angular Velocity Y:', size=(20, 1)), sg.Text(angular_velocity_y, key=ANGVELYKEY, size=(20, 1))],
    [sg.Text('Angular Velocity Z:', size=(20, 1)), sg.Text(angular_velocity_z, key=ANGVELZKEY, size=(20, 1))],
    [sg.Text('Acceleration X:', size=(20, 1)), sg.Text(f'{acceleration_x} m/s^2', key=ACCELXKEY, size=(20, 1))],
    [sg.Text('Acceleration Y:', size=(20, 1)), sg.Text(f'{acceleration_y} m/s^2', key=ACCELYKEY, size=(20, 1))],
    [sg.Text('Acceleration Z:', size=(20, 1)), sg.Text(f'{acceleration_z} m/s^2', key=ACCELZKEY, size=(20, 1))],
    [sg.Text('Magnetic Field X:', size=(20, 1)), sg.Text(f'{magnetic_field_x} uT', key=MFIELDXKEY, size=(20, 1))],
    [sg.Text('Magnetic Field Y:', size=(20, 1)), sg.Text(f'{magnetic_field_y} uT', key=MFIELDYKEY, size=(20, 1))],
    [sg.Text('Magnetic Field Z:', size=(20, 1)), sg.Text(f'{magnetic_field_z} uT', key=MFIELDZKEY, size=(20, 1))],
    [sg.Col([buttonDuo])],
]

window = sg.Window('cool GPS thingy', layout, finalize=True, resizable=True)
device_info_str = lambda device_info: f"{device_info[ID_MODEL]} ({device_info[ID_MODEL_ID]} - {device_info[ID_VENDOR_ID]})"
# Define the `on_connect` and `on_disconnect` callbacks


stop_event = Event()
serial_thread = Thread(target=serial_reader_thread, args=(window, stop_event), daemon=True)
serial_thread.start()

def on_connect(device_id, device_info):
    """Callback invoked by USB monitor thread when a device connects."""
    global USB_Found
    USB_Found = True
    print(f"Connected: {device_info_str(device_info=device_info)}")
    try:
        # Notify main loop that a device has connected
        window.write_event_value(THREAD_MESSAGE_KEY, {'status': f"Connected: {device_info_str(device_info=device_info)}"})
    except Exception:
        pass

def on_disconnect(device_id, device_info):
    """Callback invoked by USB monitor thread when a device disconnects."""
    global USB_Found, running
    USB_Found = False
    running = False
    print(f"Disconnected: {device_info_str(device_info=device_info)}")
    try:
        # Notify main loop that the device has been disconnected
        window.write_event_value(THREAD_MESSAGE_KEY, {'status': f"Disconnected: {device_info_str(device_info=device_info)}"})
    except Exception:
        pass
monitor = USBMonitor()

# Start the daemon
monitor.start_monitoring(on_connect=on_connect, on_disconnect=on_disconnect)
# Start the serial reader thread once so the GUI can detect/open the USB device
# and update the UI via THREAD_MESSAGE_KEY events.

while True:
    if serial_thread.is_alive() == False:
        if USB_Found == True:
            serial_thread = Thread(target=serial_reader_thread, args=(window, stop_event), daemon=True)
            serial_thread.start()

            
    # read event and value changes + set time (ms) for value changes + close window on exit
    event, values = window.read(timeout=100)
    if event == sg.WIN_CLOSED or event == 'Exit':
        # Signal serial thread to stop and break out of the loop
        stop_event.set()
        break

    if event == THREAD_MESSAGE_KEY:
        message = values[THREAD_MESSAGE_KEY]
        # Update GUI telemetry fields only when running (Start pressed). Status updates always update.
        if running == True:
            # Update GPS position elements if present in message
            if 'latitude' in message:
                window[LATKEY].update(message['latitude'])
            if 'longitude' in message:
                window[LONGKEY].update(message['longitude'])
            if 'elevation' in message:
                window[ELEVKEY].update(f"{message['elevation']} meters geoid")
            # Update sensor fields (acceleration, gyro, magnetic) if present
            if 'angular_velocity_x' in message:
                window[ANGVELXKEY].update(message['angular_velocity_x'])
            if 'angular_velocity_y' in message:
                window[ANGVELYKEY].update(message['angular_velocity_y'])
            if 'angular_velocity_z' in message:
                window[ANGVELZKEY].update(message['angular_velocity_z'])
            if 'acceleration_x' in message:
                window[ACCELXKEY].update(f"{message['acceleration_x']} m/s^2")
            if 'acceleration_y' in message:
                window[ACCELYKEY].update(f"{message['acceleration_y']} m/s^2")
            if 'acceleration_z' in message:
                window[ACCELZKEY].update(f"{message['acceleration_z']} m/s^2")
            if 'magnetic_field_x' in message:
                window[MFIELDXKEY].update(f"{message['magnetic_field_x']} uT")
            if 'magnetic_field_y' in message:
                window[MFIELDYKEY].update(f"{message['magnetic_field_y']} uT")
            if 'magnetic_field_z' in message:
                window[MFIELDZKEY].update(f"{message['magnetic_field_z']} uT")

        if 'status' in message:
            status_msg = message['status']

            if 'Serial read' in status_msg:
                window[USBKEY].update(status_msg)
                window['-STARTSTOP-'].update(disabled=True)
                running = False
            else:
                window[USBKEY].update(status_msg)
                if status_msg == 'Serial Port Opened.':
                    USB_Found = True
                    window['-STARTSTOP-'].update(disabled=False)
        if 'read error' in message:
            window[USBKEY].update(message)
            window['-STARTSTOP-'].update(disabled=True)
            running = False
        if 'reconnect failed' in message:
            window[USBKEY].update(message)
            window['-STARTSTOP-'].update(disabled=True)
            running = False
            break
    if USB_Found == False:
        running = False
        runningText = 'Start'
        window['-STARTSTOP-'].update(runningText)
        reset_values()
    if USB_Found == True:
        window[USBKEY].update('USB Device Found')
        window['-STARTSTOP-'].update(disabled=False)
    if event == '-STARTSTOP-':
        running = not running
        runningText = 'Stop' if running else 'Start'
        window['-STARTSTOP-'].update(runningText)
# Wait briefly for thread to exit
serial_thread.join(timeout=1)
window.close()
monitor.stop_monitoring()