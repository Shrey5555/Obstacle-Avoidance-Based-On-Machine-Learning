#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import os
import sys
import cv2
import numpy as np
import onnxruntime
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as ImageMsg
import time
from matplotlib import pyplot

def interval_mapping(image, from_min, from_max, to_min, to_max):
    from_range = from_max - from_min
    to_range = to_max - to_min
    scaled = np.array((image - from_min) / float(from_range), dtype=float)
    return to_min + (scaled * to_range)

def image_callback(img):
    global is_computing
    rospy.loginfo("computing")
    rgb_image = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
    
    #Preprocessing the image	    
    img = cv2.resize(rgb_image,(640, 480)).astype(np.float32)
    scaled_img = (img - img.min()) / (img.max() - img.min())
    img=np.array(scaled_img)
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    img = img.reshape(1,480,640,3).astype(np.float32)
    depth_image = session.run([output_name], {input_name: img})
    depth_image = np.squeeze(depth_image, axis=0)
    depth_image = np.squeeze(depth_image, axis=-1)
    depth_image = np.squeeze(depth_image, axis=0)
    depth_image = interval_mapping(depth_image, 0.0, 1.0, 0, 255).astype('uint8')
    
    # Calculate gradient
    gradient_x = cv2.Sobel(depth_image, cv2.CV_32F, 1, 0, ksize=3)
    gradient_y = cv2.Sobel(depth_image, cv2.CV_32F, 0, 1, ksize=3)
    
    # Compute surface normals
    normals_x = -gradient_x / 255.0
    normals_y = -gradient_y / 255.0
    normals_z = np.ones_like(depth_image, dtype=np.float32)
    norms = np.sqrt(normals_x**2 + normals_y**2 + normals_z**2)
    normals_x /= norms
    normals_y /= norms
    normals_z /= norms
    
    # Set threshold for vertical angle
    theta_threshold = np.pi / 3 # 60 degrees
    # Compute vertical axis vector
    vertical_axis = np.array([0, 0, 1], dtype=np.float32)
    vertical_axis = np.tile(vertical_axis, (depth_image.shape[0], depth_image.shape[1], 1))
    # Compute angle between surface normals and vertical axis
    angles = np.arccos(np.clip(np.abs(np.sum(normals_z[..., np.newaxis] * vertical_axis, axis=-1)), 0, 1))
    # Identify pixels whose surface normals have an angle greater than the threshold
    floor_mask = angles > np.deg2rad(theta_threshold)
    # Set floor pixels to 255 depth value
    depth_image[floor_mask] = 255
    depth_image = interval_mapping(depth_image, 0, 255, 0.0, 1.0)
    depth_image=depth_image*4.5 # converting depth to meters
    # Get the height and width of the image
    height, width= depth_image.shape
    
    # Split the image into left, center, and right parts
    left = depth_image[:, :width//4]
    center = depth_image[:, width//4:3*width//4]
    right = depth_image[:, 3*width//4:]

    left_half = left[height//2:, :]
    mask1 = (left_half != 4.5)
    center_third = center[height//2:, :]
    mask2 = (center_third != 4.5)
    right_half = right[height//2:, :]
    mask3 = (right_half != 4.5)

    # Threshold the depth image to detect obstacles in each region
    threshold = 0.5  # Distance threshold in meters
    left_mask = np.where(left_half < threshold, 1, 0)
    center_mask = np.where(center_third < threshold, 1, 0)
    right_mask = np.where(right_half < threshold, 1, 0)

    # Calculate the mean of the obstacle masks for each region
    left_mean = np.mean(left_mask[mask1]) 
    center_mean = np.mean(center_mask[mask2]) 
    right_mean = np.mean(right_mask[mask3])
    # If there is an obstacle in the center region, turn left or right depending on the available space
    if center_mean > 0.2:  
	if left_mean < right_mean:
	    rospy.loginfo('Turning left')
	    twist.linear.x = 0.04
	    twist.angular.z = 1
	else:
	    rospy.loginfo('Turningg right')
	    twist.linear.x = 0.04
	    twist.angular.z = -1      
    elif left_mean > 0.25:
	rospy.loginfo('Turning right')
	twist.linear.x = 0.0
	twist.angular.z = -0.5
    elif right_mean > 0.25:
	rospy.loginfo('Turning left')
	twist.linear.x = 0.0
	twist.angular.z = 0.5  # turn left
    else:
	rospy.loginfo('straight')
	twist.linear.x = 0.04
	twist.angular.z = 0.0
	
    # Publish the velocity command
    is_computing = True
    cmd_vel_pub.publish(twist)
    rospy.sleep(0.2)
    is_computing = False
    
def publish_small_linear_vel(cmd_vel_pub):
    twist = Twist()
    twist.linear.x = 0.04
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        global is_computing
        is_computing=False
        rospy.init_node('obstacle_avoidance')
        cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        decision_pub = rospy.Publisher('/decision_topic', String, queue_size=10)
        decision = "Start" 
        decision_pub.publish(decision)
        image_sub = rospy.Subscriber("/usb_camera/image_raw", ImageMsg, image_callback, queue_size = 10)
        twist = Twist()
        rate = rospy.Rate(30)  # 30 Hz
        
        # Load the ONNX model
	# Convert your model to an onnx model before hand
        path='/home/linux/Downloads/model_final.onnx'
        session = onnxruntime.InferenceSession(path, providers=['CUDAExecutionProvider'])
        input_name = session.get_inputs()[0].name
        output_name = session.get_outputs()[0].name
        print("Starting Obstacle Avoidance")
        while not rospy.is_shutdown():
           if not is_computing:
             publish_small_linear_vel(cmd_vel_pub)
           else:
             decision = "done"
             decision_pub.publish(decision)   
           rate.sleep()           
    except KeyboardInterrupt:
        rospy.loginfo("Node terminated")
        del session
    finally:
        rospy.loginfo("Node terminated")
        del session 
        

