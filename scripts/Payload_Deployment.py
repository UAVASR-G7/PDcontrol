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
PD1 = 27  # person
PD2 = 17  # backpack
PD34 = 22  # drone and phone use the same pin

# Initialize ROS node
rospy.init_node('servo_controller')
rospy.loginfo("Servo controller started")

# Time to re-zero servo positions
global zero_delay 
zero_delay = 5  # [sec]

# Function to set servo position (0 to 180 deg)
def set_servo_position(pin, angle):
    try:
        pulse_width = 500 + (angle * 2000 / 180)  # Convert angle to pulse width
        pi.set_servo_pulsewidth(pin, pulse_width)
        rospy.loginfo(f"Servo on pin {pin} set to {angle} degrees (pulse width: {pulse_width} µs)")
    except Exception as e:
        rospy.logerr(f"Failed to set servo position on pin {pin}: {str(e)}")

# Function to stop the servo signal (zero the pulse width)
def stop_servo(pin):
    rospy.loginfo(f"Stopping servo on pin {pin}")
    pi.set_servo_pulsewidth(pin, 0)  # Set pulse width to 0 to stop the servo

# Function to zero the servo position and stop the servo
def zero_servo_position(pin, zero_angle):
    rospy.loginfo(f"Zeroing servo on pin {pin} to {zero_angle} degrees")
    set_servo_position(pin, zero_angle)  # Set servo back to the specified zero angle
    Timer(0.25, stop_servo, [pin]).start()  # Stop the servo after 0.25 seconds

# Dictionary to store deployment angles
deploy_angles = {PD1: 90, PD2: 90, 'drone': 170, 'phone': 10}

# Zero positions for each servo
zero_angles = {PD1: 90, PD2: 90, PD34: 90}

# Function to initialize servos to zero position (45 degrees for PD34)
def initialize_servos():
    rospy.loginfo("Initializing servos to their zero positions...")
    zero_servo_position(PD1, zero_angles[PD1])
    zero_servo_position(PD2, zero_angles[PD2])
    zero_servo_position(PD34, zero_angles[PD34])  # Initialize PD34 (drone/phone) to 45 degrees
    rospy.sleep(rospy.Duration(2))
    rospy.loginfo("Payload Deployment Active...")

# Function to deploy payload based on the detected target
def deploy_callback(msg):
    rospy.loginfo(f"Target is: {msg.target_label}")
    
    if msg.target_label == 'person':
        set_servo_position(PD1, deploy_angles[PD1])
        rospy.loginfo(f"PD1 (person) deployed to {deploy_angles[PD1]} degrees")
        Timer(zero_delay, zero_servo_position, [PD1, zero_angles[PD1]]).start()  # Return to 0 degrees after the delay
        
    elif msg.target_label == 'backpack':
        set_servo_position(PD2, deploy_angles[PD2])
        rospy.loginfo(f"PD2 (backpack) deployed to {deploy_angles[PD2]} degrees")
        Timer(zero_delay, zero_servo_position, [PD2, zero_angles[PD2]]).start()  # Return to 0 degrees after the delay
        
    elif msg.target_label == 'drone':
        set_servo_position(PD34, deploy_angles['drone'])
        rospy.loginfo(f"PD34 (drone) deployed to {deploy_angles['drone']} degrees")
        Timer(zero_delay, zero_servo_position, [PD34, zero_angles[PD34]]).start()  # Return to 45 degrees after the delay
        
    elif msg.target_label == 'phone':
        set_servo_position(PD34, deploy_angles['phone'])
        rospy.loginfo(f"PD34 (phone) deployed to {deploy_angles['phone']} degrees")
        Timer(zero_delay, zero_servo_position, [PD34, zero_angles[PD34]]).start()  # Return to 45 degrees after the delay

# Call the initialization function before the callbacks start
initialize_servos()

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
