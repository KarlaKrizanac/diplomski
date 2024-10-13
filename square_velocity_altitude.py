#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
import tf
import math
import cv2
import threading


# Global variables to track the drone's state and position
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

# Function to send position setpoints with yaw and maintain a velocity
def set_position_with_yaw(local_position_pub, x, y, z, yaw_degrees, velocity):
    goal_pose = PoseStamped()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z

    # Convert yaw from degrees to radians and then to quaternion
    yaw_radians = math.radians(yaw_degrees)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_radians)
    goal_pose.pose.orientation.x = quaternion[0]
    goal_pose.pose.orientation.y = quaternion[1]
    goal_pose.pose.orientation.z = quaternion[2]
    goal_pose.pose.orientation.w = quaternion[3]

    # Calculate distance to the goal
    current_position_x = current_position.pose.position.x
    current_position_y = current_position.pose.position.y
    current_position_z = current_position.pose.position.z

    distance = math.sqrt((goal_pose.pose.position.x - current_position_x) ** 2 +
                         (goal_pose.pose.position.y - current_position_y) ** 2 +
                         (goal_pose.pose.position.z - current_position_z) ** 2)

    # Calculate time to reach the goal at the specified velocity
    time_to_reach = distance / velocity if velocity > 0 else 0

    local_position_pub.publish(goal_pose)
    rospy.sleep(time_to_reach)  # Wait for the calculated time to reach the point

    
    
def camera_stream():
    # Open the camera stream (you may need to adjust the index for your camera)
    cap = cv2.VideoCapture(0)
    
    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output_inverted.avi', fourcc, 20.0, (640, 480))  # Adjust resolution if needed

    while cap.isOpened() and not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            # Flip the frame vertically only
            inverted_frame = cv2.flip(frame, 0)

            # Show the live stream (inverted frame)
            cv2.imshow('Inverted Camera Stream', inverted_frame)

            # Write the vertically inverted frame to the video file
            out.write(inverted_frame)

            # Break if 'q' key is pressed (for live stream window)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    # Release everything once done
    cap.release()
    out.release()
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
    rospy.loginfo("Taking off to 10 meters altitude")
    takeoff_srv(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0)
    rospy.sleep(10)  # Stabilize at 10m

    # 1. Move to first waypoint (5, 0, 10) with yaw 0 degrees at 2 m/s
    rospy.loginfo("Moving to (5, 0, 10) at 2 m/s")
    set_position_with_yaw(local_position_pub, 5, 0, 10, 0, 2)
    rospy.sleep(3) 

    # 2. Rotate to 90 degrees and move forward by 5 meters at 1.5 m/s
    rospy.loginfo("Rotating to 90 degrees and moving forward at 1.5 m/s")
    set_position_with_yaw(local_position_pub, 5, 5, 10, 90, 1.5)
    rospy.sleep(3)

    # 3. Rotate to 180 degrees and move forward by 5 meters at 2 m/s
    rospy.loginfo("Rotating to 180 degrees and moving forward at 2 m/s")
    set_position_with_yaw(local_position_pub, 0, 5, 10, 180, 2)
    rospy.sleep(3)


    # 4. Rotate to 270 degrees and move forward by 5 meters, returning to the starting point at 1.5 m/s
    rospy.loginfo("Rotating to 270 degrees and moving forward at 1.5 m/s")
    set_position_with_yaw(local_position_pub, 0, 0, 10, 270, 1.5)
    rospy.sleep(3)


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
