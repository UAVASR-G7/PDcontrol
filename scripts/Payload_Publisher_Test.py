import rospy
from spar_msgs.msg import TargetLocalisation
from std_msgs.msg import Bool
import time

def simulated_publisher():
    # Initialize ROS node
    rospy.init_node('test_target_publisher')
    
    # Create publishers for the 'target_detection/localisation' and 'roi_status_flag' topics
    pub_target = rospy.Publisher('target_detection/localisation', TargetLocalisation, queue_size=10)
    pub_roi_flag = rospy.Publisher('/roi_status_flag', Bool, queue_size=10)
    
    # Define the targets to cycle through
    targets = ['person', 'backpack', 'drone', 'phone']
    
    rospy.loginfo("Starting continuous simulated target publisher...")
    
    # Main loop to continuously cycle through targets until shutdown
    while not rospy.is_shutdown():
        # Loop through each target in the list
        for target in targets:
            rospy.sleep(2)
            # Create and populate a TargetLocalisation message
            target_msg = TargetLocalisation()
            target_msg.target_label = target
            
            # Publish the target
            rospy.loginfo(f"Publishing target: {target}")
            pub_target.publish(target_msg)
            
            # Publish the ROI status flag as True to simulate detection
            rospy.loginfo(f"Setting ROI status flag to True for {target}")
            pub_roi_flag.publish(Bool(data=True))  # Set flag to True
            
            # Wait for the payload drop to complete
            rospy.sleep(2)  # Adjust time based on servo action timing
            
            # Reset the ROI status flag by publishing False
            rospy.loginfo("Resetting ROI status flag to False after drop")
            pub_roi_flag.publish(Bool(data=False))
            
            # Wait before the next cycle
            rospy.sleep(2)  # Wait for a short period before publishing the next target

if __name__ == '__main__':
    try:
        simulated_publisher()
    except rospy.ROSInterruptException:
        pass
