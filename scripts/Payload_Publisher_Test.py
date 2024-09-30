import rospy
from spar_msgs.msg import TargetLocalisation
import time

def simulated_publisher():
    # Initialize ROS node
    rospy.init_node('test_target_publisher')
    
    # Create a publisher for the 'target_detection/localisation' topic
    pub = rospy.Publisher('target_detection/localisation', TargetLocalisation, queue_size=10)
    
    # Define the targets to cycle through
    targets = ['person', 'backpack', 'drone', 'phone']
    
    rospy.loginfo("Starting simulated target publisher...")
    
    # Loop to publish messages at regular intervals
    rate = rospy.Rate(1)  # 1 message per second
    while not rospy.is_shutdown():
        for target in targets:
            # Create and populate a TargetLocalisation message
            msg = TargetLocalisation()
            msg.target_label = target
            
            # Publish the message
            pub.publish(msg)
            rospy.loginfo(f"Published target: {target}")
            
            # Wait for 2 seconds before sending the next target
            time.sleep(2)

if __name__ == '__main__':
    try:
        simulated_publisher()
    except rospy.ROSInterruptException:
        pass
