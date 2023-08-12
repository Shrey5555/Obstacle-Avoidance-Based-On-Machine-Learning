#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import sys
import cv2
import numpy as np
import onnxruntime
from sensor_msgs.msg import Image as ImageMsg
import time
from matplotlib import pyplot
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from math import pow, atan2, sqrt, pi,ceil
import matplotlib.pyplot as plt

def interval_mapping(image, from_min, from_max, to_min, to_max):
    from_range = from_max - from_min
    to_range = to_max - to_min
    scaled = np.array((image - from_min) / float(from_range), dtype=float)
    return to_min + (scaled * to_range)

def odom_callback(msg):
    # Get current yaw angle of the robot
    global current_yaw
    orientation = msg.pose.pose.orientation
    q0, q1, q2, q3 = orientation.w, orientation.x, orientation.y, orientation.z
    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = atan2(siny_cosp, cosy_cosp)
    current_yaw = yaw
    
def depth_callback(img):
    global is_computing,rotate,c,prev,check,cnt1
    if Match==True or check==True or rotate==True or prev==True:
    	return 
    cnt1+=1
    if cnt1%10==0 or cnt1==1:
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
	    theta_threshold = np.pi / 3.5 # 60 degrees
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
	    depth_image=depth_image*4.5# converting depth to meters
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
	    
	    if left_mean > 0.25:
	      rospy.loginfo('right')
	      twist.linear.x = 0.0
	      twist.angular.z = -0.8

	    elif right_mean > 0.25:
	      rospy.loginfo('left')
	      twist.linear.x = 0.0
	      twist.angular.z = 0.8  
	    elif center_mean > 0.2:   
               if left_mean < right_mean:
                    rospy.loginfo('Turning left')
                    twist.linear.x = 0.05
                    twist.angular.z = 1
               else:
                    rospy.loginfo('Turning right')
                    twist.linear.x = 0.05
                    twist.angular.z = -1          
	    else:
	      rospy.loginfo('straight')
	      twist.linear.x = 0.05
	      twist.angular.z = 0.0
	    c+=1  
	    is_computing = True
	    cmd_vel_pub.publish(twist)
	    rospy.sleep(0.2)
	    is_computing = False
	    if c==10:
	      rotate=True
	      c=0
    else:
      pass
    
def image_callback(img):
    global rgb_image, counter, rotate,prev,check, percentage_match,count,cnt2,end,cc
    cnt2+=1
    if cnt2%10==0 or cnt2==1:
	    start=time.time()
	    rgb_image = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
	    global target_pixel, Match, goal_img,c,obstacle_detected,dst_pts_good,is_computing
	    global depth_image,is_computing, Match, c,turn_countl,turn_countr,prev
	    # Convert both images to grayscale
	    gray2 = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
	    kp2, des2 = sift.detectAndCompute(gray2, None)
	    # Create matcher object
	    bf = cv2.BFMatcher()
	    # Match descriptors of the two images
	    match = bf.knnMatch(des1, des2, k=2)
	    matches = []
	    for m, n in match:
               if m.distance < 0.6 * n.distance:
                   matches.append(m)
	    if len(matches) < 20:
              Match= False
              print("No matches found in the image") 
	    else:
                # Apply RANSAC algorithm to filter out outliers
                src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
                # Calculate the homography matrix using RANSAC algorithm
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                ht,wt = 240,432
                points=np.float32([[0, 0], [0, ht], [wt, ht], [wt, 0]]).reshape(-1, 1, 2)
                dst_points = cv2.perspectiveTransform(points, M)
                dst_pts_good = dst_pts[mask.ravel() == 1]
                homography = cv2.polylines(rgb_image, [np.int32(dst_points)], True, (255, 0, 0), 3)
                dst_points=dst_points.reshape(4,2)
                a = dst_points[0,0]
                b = dst_points[1,1]
                c = dst_points[0,1]
                d = dst_points[3,0]
                x,y = dst_points[0,0],dst_points[0,1]
                w = int(d-a)
                h = int(b-c)
                matches = [matches[i] for i in range(len(matches)) if mask[i]]
                print("2",len(matches))
                if len(matches) > 15:
                    Match= True
                    rotate=False
                    prev=True
                    target_pixel = (int(x + w/2), int(y + h/2))
                    num_good_matches = len(dst_pts_good)
                    print(num_good_matches)
                    print(len(kp1))
                    total_matches = len(kp1)
                    percentage_match = num_good_matches / total_matches * 100
                    print("Percentage of match: {:.2f}%".format(percentage_match))
                    print("Average matched point: ({}, {})".format(int(x + w/2), int(y + h/2)))
                    rospy.loginfo("Moving towards Goal")
                    plt.imshow(homography)
                    img_n="image_{}.png".format(cc)
                    img_path="/home/linux/Downloads/rec"
                    cv2.imwrite(os.path.join(img_path,img_n), homography,[cv2.IMWRITE_PNG_COMPRESSION,0])
                    cc+=1
                    if (21 < percentage_match < 100):
                          count += 1
                          if count == 3:
                             print("REACHED")
                             end=True
                    go_to_goal(target_pixel)
                else:
                    Match= False
                    print("No matches found in the image")            
	    if Match==False and rotate==True:
                is_computing=True
                twist = Twist()
                twist.angular.z = -0.7 # 15 degrees
                cmd_vel_pub.publish(twist)
                counter+=1
                if counter==25:
                    is_computing=False
                    rotate=False
	    end=time.time()
	    print("time=", end-start)
    else:
      pass
            
def go_to_goal(target_pixel):
    global linear_velocity, angular_velocity,prev_yaw,turn_countr,turn_countl,is_computing,check
    x_diff = (target_pixel[0]) - image_width/2
    is_computing=True
    twist = Twist()
    if x_diff > 5:
       twist.angular.z = -angular_velocity
    elif x_diff < -5:
       twist.angular.z = angular_velocity
    else:
       twist.angular.z = 0.0
    twist.linear.x = linear_velocity
    cmd_vel_pub.publish(twist)
    rospy.sleep(0.2)
    is_computing=False
    check=check_obstacle()
    if check==True:
        rospy.loginfo('Moving Around Obstacle')
        prev_yaw=current_yaw
        move_around()
        while 1:
            rospy.loginfo('Reverting to previous angle')
            if turn_countr>turn_countl:
                twist.linear.x =0.0
                twist.angular.z = 0.2
            else:
                twist.linear.x = 0.0
                twist.angular.z = -0.2
            time.sleep(0.5)
            is_computing = True
            cmd_vel_pub.publish(twist)
            is_computing = False
            if abs(prev_yaw-current_yaw) < 0.05:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                cmd_vel_pub.publish(twist)
                break
        
def move_around():
    while 1:
        global rgb_image,prev_yaw,turn_countr,turn_countl,is_computing
        is_computing = False
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
        theta_threshold = np.pi / 3.5 # 60 degrees
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
        depth_image=depth_image*4.5# converting depth to meters
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
        if left_mean > 0.25:
            rospy.loginfo('right')
            twist.linear.x = 0.0
            twist.angular.z = -0.8
            turn_countr+=1
        elif right_mean > 0.25:
            rospy.loginfo('left')
            twist.linear.x = 0.0
            twist.angular.z = 0.8  # turn left
            turn_countl+=1
        elif center_mean > 0.2:
            if left_mean < right_mean:
                    rospy.loginfo('Turning left')
                    twist.linear.x = 0.05
                    twist.angular.z = 1
                    turn_countl+=1
            else:
                    rospy.loginfo('Turning right')
                    twist.linear.x = 0.05
                    twist.angular.z = -1
                    turn_countr+=1
            else:
                break
            
        is_computing = True
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.2)
        is_computing = False
	    
        
def check_obstacle():
    global rgb_image, linear_velocity, angular_velocity,is_computing
    is_computing = False
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
    theta_threshold = np.pi / 3.5 # 60 degrees
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
    depth_image=depth_image*4.5# converting depth to meters
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
    left_m = np.mean(left_mask[mask1]) 
    center_m = np.mean(center_mask[mask2]) 
    right_m= np.mean(right_mask[mask3])
    # If there is an obstacle in the center region, turn left or right depending on the available space
    if center_m> 0.1:
        return True
    elif left_m > 0.1:
        return True
    elif right_m > 0.1:
        return True
    else:
        return False
def publish_small_linear_vel(cmd_vel_pub):
    twist = Twist()
    twist.linear.x = 0.04
    cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        global counter,turn_countl,turn_countr, linear_velocity, angular_velocity,dst_pts_good,rotate,cnt1,cnt2,end
        global is_computing,Match,goal_img,obstacle_detected,x,percentage_match,initial_yaw,c,prev,check,cc
        cc=0
        end=False
        check=False
        prev=False
        percentage_match=0
        cnt1=0
        cnt2=0
        c=0
        count=0
        goal_img=cv2.imread("/home/linux/Downloads/image_goal_final.png")
        #goal_img = cv2.resize(goal_img,(640, 480))
        # Create SIFT detector
        sift = cv2.xfeatures2d.SIFT_create()
        gray1 = cv2.cvtColor(goal_img, cv2.COLOR_BGR2GRAY)
        # Detect keypoints and compute descriptors for both images
        kp1, des1 = sift.detectAndCompute(gray1, None)
        dst_pts_good=[]
        Match=False
        rotate=True
        obstacle_detected=False
        initial_yaw=0.0
        counter=0
        turn_countl=0
        turn_countr=0
        is_computing=False
        current_yaw = 0.0
        linear_velocity = 0.04
        angular_velocity = 0.3
        image_width = 432
        image_height = 240
        rospy.init_node('obstacle_avoidance')
        image_sub = rospy.Subscriber("/usb_camera/image_raw", ImageMsg, image_callback)
        image_sub1 = rospy.Subscriber("/usb_camera/image_raw", ImageMsg, depth_callback)
        cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
        initial_yaw=current_yaw
        twist = Twist()
        rate = rospy.Rate(10)#1Hz
        # Load the ONNX model
        path='/home/linux/Downloads/model_final.onnx'
        session = onnxruntime.InferenceSession(path, providers=['CPUExecutionProvider'])
        input_name = session.get_inputs()[0].name
        output_name = session.get_outputs()[0].name
        print("Starting Obstacle Avoidance")
        # Continuously publish the small linear velocity until the callback function starts computing
        while not rospy.is_shutdown():
            if not is_computing:
               publish_small_linear_vel(cmd_vel_pub)
            elif end==True:
               twist = Twist()
               twist.linear.x = 0.0
               twist.angular.z=0.0
               cmd_vel_pub.publish(twist)
               break
            rate.sleep()           
        rospy.shutdown(image_callback)
    except KeyboardInterrupt:
        del session
        sys.exit("Node terminated")
    finally:
        del session
        sys.exit("Node terminated")
        




