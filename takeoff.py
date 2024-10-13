#!/usr/bin/env python3

import rospy
import subprocess
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
import time

# Global variable to track the drone's state
current_state = None

# Callback function to update the drone's current state
def state_cb(msg):
    global current_state
    current_state = msg

def main():
    # Initialize ROS node
    rospy.init_node('drone_control')

    # ROS Subscribers and Service Proxies
    state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
    set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

    # Ensure the services are available with timeout
    rospy.wait_for_service('/mavros/cmd/arming', timeout=10)
    rospy.wait_for_service('/mavros/set_mode', timeout=10)
    rospy.wait_for_service('/mavros/cmd/takeoff', timeout=10)
    rospy.wait_for_service('/mavros/cmd/land', timeout=10)


    rate = rospy.Rate(20)  # 20hz

    # Wait for state to be populated
    while not rospy.is_shutdown() and current_state is None:
        rospy.loginfo("Waiting for state information...")
        rate.sleep()

    # Wait for FCU (Flight Control Unit) connection
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rate.sleep()

    # Arm the drone
    rospy.loginfo("Arming the drone...")
    if not arming_srv(True).success:
        rospy.logerr("Failed to arm the drone!")
        return

    # Set mode to GUIDED
    rospy.loginfo("Setting mode to GUIDED")
    if not set_mode_srv(custom_mode="GUIDED").mode_sent:
        rospy.logerr("Failed to set GUIDED mode!")
        return

    # Wait a bit to ensure the mode is set
    rospy.sleep(2)

    # Takeoff to 10 meters
    rospy.loginfo("Taking off to 10m altitude")
    if not takeoff_srv(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0).success:
        rospy.logerr("Failed to take off!")
        return
    # Hover for 10 seconds
    #rospy.loginfo("Hovering...")
    #rospy.sleep(10)
    
    # Land the drone
    rospy.loginfo("Landing")
    if not land_srv(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0).success:
        rospy.logerr("Failed to land!")
        return

    # Disarm after landing
    rospy.loginfo("Disarming the drone after landing.")
    arming_srv(False)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

