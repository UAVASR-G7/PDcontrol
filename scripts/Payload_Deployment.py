import pigpio
import rospy
from spar_msgs.msg import TargetLocalisation
from threading import Timer

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    rospy.logerr("Failed to connect to pigpio daemon!")
    exit(1)

# Define GPIO pins for the servos
PD1 = 27 # person
PD2 = 22 # backpack
PD34 = 17 #  drone & phone

# Initialize ROS node
rospy.init_node('servo_controller')
rospy.loginfo("Servo controller started")

# Time to re-zero servo positions
global zero_delay 
zero_delay = 5 # [sec]

# Function to set servo position
def set_servo_position(pin, angle):
    try:
        pulse_width = 500 + (angle * 2000 / 180)  # Convert angle to pulse width
        pi.set_servo_pulsewidth(pin, pulse_width)
    except Exception as e:
        rospy.logerr(f"Failed to set servo position on pin {pin}: {str(e)}")

# Function to stop the servo signal (zero the pulse width)
def stop_servo(pin):
    rospy.loginfo(f"Stopping servo on pin {pin}")
    pi.set_servo_pulsewidth(pin, 0)  # Set pulse width to 0 to stop the servo

# Function to zero the servo position after 10 seconds and stop the servo
def zero_servo_position(pin):
    rospy.loginfo(f"Zeroing and stopping servo on pin {pin} after 10 seconds")
    set_servo_position(pin, 0)  # Zero the servo
    Timer(0.25, stop_servo, [pin]).start()  # Stop the servo 2 seconds after zeroing

# Dictionary to store the last angles and deployment angles
deploy_angles = {PD1: 90, PD2: 90, PD34: 90}  # Default deployment angles

# Function to deploy payload based on the detected target
def deploy_callback(msg):
    rospy.loginfo(f"Target is: {msg.target_label}")
    
    if msg.target_label == 'person':
        set_servo_position(PD1, deploy_angles[PD1])
        rospy.loginfo(f"PD1 (person) deployed to {deploy_angles[PD1]} degrees")
        Timer(zero_delay, zero_servo_position, [PD1]).start()  # Zero after 10 seconds
        
    elif msg.target_label == 'backpack':
        set_servo_position(PD2, deploy_angles[PD2])
        rospy.loginfo(f"PD2 (backpack) deployed to {deploy_angles[PD2]} degrees")
        Timer(zero_delay, zero_servo_position, [PD2]).start()  # Zero after 10 seconds
        
    elif msg.target_label == 'drone':
        set_servo_position(PD34, deploy_angles[PD34])
        rospy.loginfo(f"PD34 (drone) deployed to {deploy_angles[PD34]} degrees")
        Timer(zero_delay, zero_servo_position, [PD34]).start()  # Zero after 10 seconds
        
    elif msg.target_label == 'phone':
        set_servo_position(PD34, deploy_angles[PD34])
        rospy.loginfo(f"PD34 (phone) deployed to {deploy_angles[PD34]} degrees")
        Timer(zero_delay, zero_servo_position, [PD34]).start()  # Zero after 10 seconds

# ROS topic subscriber for target detection
rospy.Subscriber('target_detection/localisation', TargetLocalisation, deploy_callback)

# Keep the node running
rospy.spin()

# Cleanup pigpio and shut down GPIO on shutdown
def shutdown_handler():
    rospy.loginfo("Shutting down servo controller...")
    stop_servo(PD1)
    stop_servo(PD2)
    stop_servo(PD34)
    pi.stop()  # Cleanup pigpio

rospy.on_shutdown(shutdown_handler)
