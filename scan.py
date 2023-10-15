"""This module is for serial communication with the robot."""
from time import strftime, localtime
import serial

# Set up serial communication

# Use `with` statement to avoid unclosed serial communication after the end of
# program.
with serial.Serial() as robot:
    # Initialize the robot with the specifics
    robot.port = "/dev/tty.usbmodem1101"  # Replace with actual port
    robot.baudrate = 9600  # This must match baudrate of Arduino.

    # Attempt to open the serial connection
    try:
        robot.open()
    except serial.serialutil.SerialException:
        print("No robot found on the port. Try again with a different port.")

    # Attempt to read data from serial connection
    try:
        # Grab current time for file name
        time_prefix = strftime("%a-%d-%b-%Y-%H-%M", localtime())
        # Continue to run as long as the robot's connection is open
        while robot.is_open:
            # Read and parse incoming data from the Arduino
            data = str(robot.readline().decode().strip())

            # Assuming that the data from Arduino will look something like
            # "M_L,M_R,S_L,S_M,S_R".
            print(data)
            # Save the data into a local file (maybe a .csv file)
            with open(
                f"{time_prefix}.csv", "a", newline="\n", encoding="utf-8"
            ) as f:
                f.write(data)
                f.write("\n")

    except KeyboardInterrupt:
        robot.close()  # Close the serial port on Ctrl+C
