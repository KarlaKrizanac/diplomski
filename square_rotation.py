#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
import cv2
import threading

# Global variable to track the drone's state and position
current_state = None
current_position = PoseStamped()

# Callback function to update the drone's current state
def state_cb(msg):
    global current_state
    current_state = msg

# Callback function to track current position
def position_cb(msg):
    global current_position
    current_position = msg

# Function to send position setpoints
def set_position(local_position_pub, x, y, z):
    goal_pose = PoseStamped()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z
    local_position_pub.publish(goal_pose)

    # Wait for the drone to reach the target position
    rospy.sleep(3)  # Adjust based on the desired hover time

def camera_stream():
    cap = cv2.VideoCapture(0)  # Access the default camera
    if not cap.isOpened():
        rospy.logerr("Cannot open camera")
        return

    # Set the desired resolution (e.g., 640x480)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to grab frame from camera")
            break
        
        flipped_frame = cv2.flip(frame, 0)
        cv2.imshow('Camera Stream', flipped_frame)
        cv2.waitKey(1)  # Required to update the display

    cap.release()
    cv2.destroyAllWindows()

def main():
    # Initialize ROS node
    rospy.init_node('drone_control')

    # ROS Subscribers and Service Proxies
    state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
    position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, position_cb)
    set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
    
    local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # Ensure the services are available with timeout
    rospy.wait_for_service('/mavros/cmd/arming', timeout=10)
    rospy.wait_for_service('/mavros/set_mode', timeout=10)
    rospy.wait_for_service('/mavros/cmd/takeoff', timeout=10)
    rospy.wait_for_service('/mavros/cmd/land', timeout=10)

    rate = rospy.Rate(20)  # 20hz

    # Wait for state to be populated and FCU connection
    while not rospy.is_shutdown() and (current_state is None or not current_state.connected):
        rospy.loginfo("Waiting for state information and FCU connection...")
        rate.sleep()

    # Arm the drone
    rospy.loginfo("Arming the drone...")
    arming_srv(True)

    # Set mode to GUIDED
    rospy.loginfo("Setting mode to GUIDED")
    set_mode_srv(custom_mode="GUIDED")

    rospy.sleep(2)

    # Start the camera stream in a separate thread
    camera_thread = threading.Thread(target=camera_stream)
    camera_thread.start()

    # Takeoff to 10 meters
    rospy.loginfo("Taking off to 10m altitude")
    takeoff_srv(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0)

    rospy.sleep(10)  # Stabilize at 10m before starting the pattern

    # 1. Move forward to (5, 0, 10)
    rospy.loginfo("Moving to (5, 0, 10)")
    set_position(local_position_pub, 5, 0, 10)
    rospy.sleep(3)  # Hover for 3 seconds

    # 2. Move left to (5, 5, 10)
    rospy.loginfo("Moving to (5, 5, 10)")
    set_position(local_position_pub, 5, 5, 10)
    rospy.sleep(3)  # Hover for 3 seconds

    # 3. Move back to (0, 5, 10)
    rospy.loginfo("Moving to (0, 5, 10)")
    set_position(local_position_pub, 0, 5, 10)
    rospy.sleep(3)  # Hover for 3 seconds

    # Return to start point (0, 0, 10)
    rospy.loginfo("Returning to start point (0, 0, 10)")
    set_position(local_position_pub, 0, 0, 10)
    rospy.sleep(3)  # Hover for 3 seconds before landing

    # Land the drone
    rospy.loginfo("Landing the drone")
    land_srv(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
    
    # Disarm after landing
    rospy.loginfo("Disarming the drone after landing.")
    arming_srv(False)

    # Wait for the camera thread to finish
    camera_thread.join()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

