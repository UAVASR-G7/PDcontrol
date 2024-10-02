import pigpio
import rospy
from spar_msgs.msg import TargetLocalisation
from std_msgs.msg import Bool  # Ensure this is imported for roi_status_flag
from threading import Timer

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    rospy.logerr("Failed to connect to pigpio daemon!")
    rospy.signal_shutdown("Failed to connect to pigpio daemon!")  # Graceful shutdown

# Define GPIO pins for the servos
PD1 = 17  # person
PD2 = 27  # backpack
PD34 = 22  # drone and phone use the same pin

# Initialize ROS node
rospy.init_node('servo_controller')
rospy.loginfo("Servo controller started")

# Time to re-zero servo positions
global zero_delay
zero_delay = 5  # [sec]

# Initialize roi_status_flag (this variable is used to check that the /roi_status_callback topic is in fact True, for the payload dropping)
roi_status_flag = False

# Initialise TargetID variable (stores the target ID as a global variable to pass between callbacks)
TargetID = "Unset_ID"

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
deploy_angles = {PD1: 60, PD2: 120, 'drone': 160, 'phone': 20}

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

# Store the TargetID as a global variable (to reference in roi_status_callback)
def TargetID_callback(msg_in):
    global TargetID
    TargetID = msg_in.target_label

# roi status callback (This deploys the payload when the roi status flag is set to true, see demo_wp_roi)
# This determines which payload to drop depending on the TargetID global variable, which is set in TargetID_callback() function
def roi_status_callback(msg):
    global TargetID

    if msg.data:  # only initiate this section on a recieved True
        rospy.loginfo(f"ROI Status Callback Triggered for Target: {TargetID}")
        
        # Perform the payload deployment based on the TargetID
        if TargetID == 'person':
            set_servo_position(PD1, deploy_angles[PD1])
            rospy.loginfo(f"PD1 (person) deployed to {deploy_angles[PD1]} degrees")
            Timer(zero_delay, zero_servo_position, [PD1, zero_angles[PD1]]).start()

        elif TargetID == 'backpack':
            set_servo_position(PD2, deploy_angles[PD2])
            rospy.loginfo(f"PD2 (backpack) deployed to {deploy_angles[PD2]} degrees")
            Timer(zero_delay, zero_servo_position, [PD2, zero_angles[PD2]]).start()

        elif TargetID == 'drone':
            set_servo_position(PD34, deploy_angles['drone'])
            rospy.loginfo(f"PD34 (drone) deployed to {deploy_angles['drone']} degrees")
            Timer(zero_delay, zero_servo_position, [PD34, zero_angles[PD34]]).start()
        
        elif TargetID == 'phone':
            set_servo_position(PD34, deploy_angles['phone'])
            rospy.loginfo(f"PD34 (phone) deployed to {deploy_angles['phone']} degrees")
            Timer(zero_delay, zero_servo_position, [PD34, zero_angles[PD34]]).start()

        # Reset TargetID after deployment for debugging
        TargetID = "Unset_ID"


# Call the initialization function before the callbacks start
initialize_servos()

# ROS topic subscriber for target detection
rospy.Subscriber('target_detection/localisation', TargetLocalisation, TargetID_callback)

# rostopic subscriber for roi target status
rospy.Subscriber('/roi_status_flag', Bool, roi_status_callback)

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
