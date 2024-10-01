import pigpio
import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from spar_msgs.msg import TargetLocalisation
import os

def start():
    # Initialize pigpio
    pi = pigpio.pi()

    # Define GPIO pins for the servos
    PD1 = 17
    PD2 = 27
    PD34 = 22

    # Initialize ROS node
    rospy.init_node('servo_controller')
    rospy.loginfo("started")

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
                set_servo_position(pin, angle)
                deploy_angles[pin] = angle
            except ValueError:
                command = input("Type 'next' to move to the next servo or 'exit' to finish: ").strip().lower()
                if command == 'next':
                    break
                elif command == 'exit':
                    return 'exit'

    # Function to print servo states
    def print_servo_states():
        rospy.loginfo(f"Servo States - PD1: {last_angles[PD1]} (Initial), {deploy_angles[PD1]} (Deployment), PD2: {last_angles[PD2]} (Initial), {deploy_angles[PD2]} (Deployment), PD34: {last_angles[PD34]} (Initial), {deploy_angles[PD34]} (Deployment)")

    # Payload deployment functions
    def servo_setup(_):
        if get_servo_angle(PD1) == 'exit':
            return
        if get_servo_angle(PD2) == 'exit':
            return
        get_servo_angle(PD34)
        write_servo_values()
        print_servo_states()
        return EmptyResponse()

    def zero_servos(_):
        global last_angles
        set_servo_position(PD1, 90)
        set_servo_position(PD2, 90)
        set_servo_position(PD34, 130)
        print_servo_states()
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
        print_servo_states()
        return EmptyResponse()

    def deployPD1():
        rospy.loginfo("PD1 COMMAND REVEIVED")
        # rospy.sleep(rospy.Duration(10))
        set_servo_position(PD1, 30)
        print_servo_states()
        

    def deployPD2():
        rospy.loginfo("PD2 COMMAND REVEIVED")
        # rospy.sleep(rospy.Duration(10))
        set_servo_position(PD2, 30)
        print_servo_states()
        

    def deployPD3():
        set_servo_position(PD34, 30)
        print_servo_states()

    def deployPD4():
        set_servo_position(PD34, 130)
        print_servo_states()
        
    def moveall(_):
        set_servo_position(PD1, 45)
        set_servo_position(PD2, 45)
        set_servo_position(PD34, 90)
        rospy.loginfo("All servos moved to 90 degrees")
        print_servo_states()


    # Read servo values from file at startup
    read_servo_values()

    # ROS service servers
    rospy.Service('servo_setup', Empty, servo_setup)
    rospy.Service('zero_servos', Empty, zero_servos)
    rospy.Service('fullreset', Empty, fullreset)
    rospy.Service('moveall', Empty, moveall)
    #rospy.Service('pd1', Empty, deployPD1)
    #rospy.Service('pd2', Empty, deployPD2)
    #rospy.Service('pd3', Empty, deployPD3)
    #rospy.Service('pd4', Empty, deployPD4)

    # ROS topic subscriber callback
    def deploy_callback(msg):
        rospy.loginfo(f"Target is: {msg.target_label}")
        if msg.target_label == "person":
            deployPD1()
            rospy.loginfo(f"deployed person package")
        elif msg.target_label == 'backpack':
            deployPD2()
            rospy.loginfo(f"deployed backpack package")
        elif msg.target_label == 'drone':
            deployPD3()
        elif msg.target_label == 'phone':
            deployPD4()

    # ROS topic subscriber
    #rospy.Subscriber('payload/target', String, deploy_callback)
    rospy.Subscriber('target_detection/localisation', TargetLocalisation, deploy_callback)

    # Keep the node running
    # rospy.spin()

    # Cleanup
    # pi.stop()

if __name__ == '__main__':
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass