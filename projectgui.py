import FreeSimpleGUI as sg
import time
import random
import csv
with open('gpsdata.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['Date','Time', 'Satellites', 'Latitude', 'Longitude', 'Elevation', 'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z',
                     'Acceleration X (m/s^2)', 'Acceleration Y (m/s^2)', 'Acceleration Z (m/s^2)',
                     'Magnetic Field X (uT)', 'Magnetic Field Y (uT)', 'Magnetic Field Z (uT)'])
    
def write_to_csv(date, time_str, satellites, latitude, longitude, elevation,
                 ang_vel_x, ang_vel_y, ang_vel_z,
                 accel_x, accel_y, accel_z,
                 mag_field_x, mag_field_y, mag_field_z):
    with open('gpsdata.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([date, time_str, satellites, latitude, longitude, elevation,
                         ang_vel_x, ang_vel_y, ang_vel_z,
                         accel_x, accel_y, accel_z,
                         mag_field_x, mag_field_y, mag_field_z])
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
# Initial placeholder values
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
USB_Found = True
USB_Status = 'No USB Device Found'

starting_time = time.time()

buttonDuo=[sg.Button(runningText, key='-STARTSTOP-',disabled=not USB_Found), sg.Button('Exit'), sg.Text(USB_Status, key=USBKEY)]
layout = [
    [sg.Text('Latitude:', size=(20, 1)), sg.Text(latitude_value, key=LATKEY, size=(10, 1))],
    [sg.Text('Longitude:', size=(20, 1)), sg.Text(longitude_value, key=LONGKEY, size=(10, 1))],
    [sg.Text('Elevation:', size=(20, 1)), sg.Text(elevation_value, key=ELEVKEY, size=(10, 1))],
    [sg.Text('Angular Velocity X:', size=(20, 1)), sg.Text(angular_velocity_x, key=ANGVELXKEY, size=(10, 1))],
    [sg.Text('Angular Velocity Y:', size=(20, 1)), sg.Text(angular_velocity_y, key=ANGVELYKEY, size=(10, 1))],
    [sg.Text('Angular Velocity Z:', size=(20, 1)), sg.Text(angular_velocity_z, key=ANGVELZKEY, size=(10, 1))],
    [sg.Text('Acceleration X:', size=(20, 1)), sg.Text(f'{acceleration_x} m/s^2', key=ACCELXKEY, size=(10, 1))],
    [sg.Text('Acceleration Y:', size=(20, 1)), sg.Text(f'{acceleration_y} m/s^2', key=ACCELYKEY, size=(10, 1))],
    [sg.Text('Acceleration Z:', size=(20, 1)), sg.Text(f'{acceleration_z} m/s^2', key=ACCELZKEY, size=(10, 1))],
    [sg.Text('Magnetic Field X:', size=(20, 1)), sg.Text(f'{magnetic_field_x} uT', key=MFIELDXKEY, size=(10, 1))],
    [sg.Text('Magnetic Field Y:', size=(20, 1)), sg.Text(f'{magnetic_field_y} uT', key=MFIELDYKEY, size=(10, 1))],
    [sg.Text('Magnetic Field Z:', size=(20, 1)), sg.Text(f'{magnetic_field_z} uT', key=MFIELDZKEY, size=(10, 1))],
    [sg.Col([buttonDuo])],
]

window = sg.Window('cool GPS thingy', layout)
while True:
    
    # read event and value changes + set time (ms) for value changes + close window on exit
    event, values = window.read(timeout=100)
    if event == sg.WIN_CLOSED or event == 'Exit':
        break


    if event == sg.TIMEOUT_KEY and running:
        # Calculate the new value of the Python variable
        current_time = time.time()
        time_elapsed = current_time - starting_time
        write_to_csv(time.strftime("%Y-%m-%d"), time.strftime("%H:%M:%S"), '8', f'{random.uniform(-90.0, 90.0):.6f}', f'{random.uniform(-180.0, 180.0):.6f}', f'{random.uniform(0, 1000):.2f}',
                     f'{random.uniform(-500.0, 500.0):.2f}', f'{random.uniform(-500.0, 500.0):.2f}', f'{random.uniform(-500.0, 500.0):.2f}',
                     f'{random.uniform(-10.0, 10.0):.2f}', f'{random.uniform(-10.0, 10.0):.2f}', f'{random.uniform(-10.0, 10.0):.2f}',
                     f'{random.uniform(-100.0, 100.0):.2f}', f'{random.uniform(-100.0, 100.0):.2f}', f'{random.uniform(-100.0, 100.0):.2f}')
        # 4. Update the Text Element with the new variable value
        # Use a formatted string to display two decimal places
        new_text = f'{time_elapsed:.2f}'

        # Access the element using window[KEY] and call .update(new_text)
        window[LATKEY].update(new_text)
    if USB_Found == True:
        window[USBKEY].update('USB Device Found')
    if event == '-STARTSTOP-':
        running = not running
        runningText = 'Stop' if running else 'Start'
        window['-STARTSTOP-'].update(runningText)

window.close()