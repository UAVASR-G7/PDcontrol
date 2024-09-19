import pigpio
import rospy
from std_srvs.srv import Empty, EmptyResponse
import os

# Initialize pigpio
pi = pigpio.pi()

# Define GPIO pins for the servos
PD1 = 27
PD2 = 22
PD34 = 13

# Initialize ROS node
rospy.init_node('servo_controller')

# Function to set servo position
def set_servo_position(pin, angle):
    pulse_width = 500 + (angle * 2000 / 180)
    pi.set_servo_pulsewidth(pin, pulse_width)

# Dictionary to store the last angles and deployment angles
last_angles = {PD1: 0, PD2: 0, PD34: 0}
deploy_angles = {PD1: 0, PD2: 0, PD34: 0}

# Function to read servo values from file
def read_servo_values():
    global last_angles, deploy_angles
    if os.path.exists('servo_values.txt'):
        with open('servo_values.txt', 'r') as file:
            lines = file.readlines()
            for line in lines:
                pin, last_angle, deploy_angle = map(float, line.strip().split(','))
                last_angles[int(pin)] = last_angle
                deploy_angles[int(pin)] = deploy_angle

# Function to write servo values to file
def write_servo_values():
    with open('servo_values.txt', 'w') as file:
        for pin in last_angles:
            file.write(f"{pin},{last_angles[pin]},{deploy_angles[pin]}\n")

# Function to get user input for servo angles
def get_servo_angle(pin):
    while True:
        try:
            angle = float(input(f"Enter initial angle for servo on pin {pin} (or type 'next' to move to the next servo): "))
            set_servo_position(pin, angle)
            last_angles[pin] = angle
        except ValueError:
            command = input("Type 'next' to move to the next servo or 'exit' to finish: ").strip().lower()
            if command == 'next':
                break
            elif command == 'exit':
                return 'exit'
    
    while True:
        try:
            angle = float(input(f"Enter deployment angle for servo on pin {pin} (or type 'next' to move to the next servo): "))
            deploy_angles[pin] = angle
            break
        except ValueError:
            print("Invalid input. Please enter a valid number.")

# Payload deployment functions
def servo_setup(_):
    if get_servo_angle(PD1) == 'exit':
        return
    if get_servo_angle(PD2) == 'exit':
        return
    get_servo_angle(PD34)
    write_servo_values()
    return EmptyResponse()

def zero_servos(_):
    global last_angles
    set_servo_position(PD34, last_angles[PD34])
    set_servo_position(PD1, last_angles[PD1])
    set_servo_position(PD2, last_angles[PD2])
    return EmptyResponse()

def fullreset(_):
    global last_angles, deploy_angles
    last_angles = {PD1: 0, PD2: 0, PD34: 0}
    deploy_angles = {PD1: 0, PD2: 0, PD34: 0}
    set_servo_position(PD34, 0)
    set_servo_position(PD1, 0)
    set_servo_position(PD2, 0)
    # Wipe the servo_values.txt file
    if os.path.exists('servo_values.txt'):
        os.remove('servo_values.txt')
    return EmptyResponse()

def deployPD1(_):
    set_servo_position(PD1, deploy_angles[PD1])
    return EmptyResponse()

def deployPD2(_):
    set_servo_position(PD2, deploy_angles[PD2])
    return EmptyResponse()

def deployPD3(_):
    set_servo_position(PD34, deploy_angles[PD34])
    return EmptyResponse()

def deployPD4(_):
    set_servo_position(PD34, deploy_angles[PD34])
    return EmptyResponse()

# Read servo values from file at startup
read_servo_values()

# ROS service servers
rospy.Service('servo_setup', Empty, servo_setup)
rospy.Service('zero_servos', Empty, zero_servos)
rospy.Service('fullreset', Empty, fullreset)
rospy.Service('deployPD1', Empty, deployPD1)
rospy.Service('deployPD2', Empty, deployPD2)
rospy.Service('deployPD3', Empty, deployPD3)
rospy.Service('deployPD4', Empty, deployPD4)

# Keep the node running
rospy.spin()

# Cleanup
pi.stop()
