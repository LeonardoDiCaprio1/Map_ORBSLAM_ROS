import numpy
import rosbag
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

rgb_path = '/home/zhao/liu/script/rgb/'
depth_path = '/home/zhao/liu/script/depth/'
bridge = CvBridge()

num = 1

with rosbag.Bag('/home/zhao/liu/script/image.bag', 'r') as bag:
    for topic, msg, t in bag.read_messages():
        if topic == "/camera/depth/image_raw":
            cv_image = bridge.imgmsg_to_cv2(msg, '32FC1')
            cv_image = cv_image * 255  

            image_name = str(num) + '.png'
            cv2.imwrite(depth_path + image_name, cv_image)  
            print(depth_path + image_name)

            timestr = "%.8f" % msg.header.stamp.to_sec()
            with open('depth.txt', 'a') as depth_time_file:
                depth_time_file.write(timestr + " depth/" + image_name + "\n")

        if topic == "/camera/rgb/image_raw": 
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            image_name = str(num) + '.png'
            cv2.imwrite(rgb_path + image_name, cv_image)

            timestr = "%.8f" % msg.header.stamp.to_sec()
            with open('rgb.txt', 'a') as rgb_time_file:
                rgb_time_file.write(timestr + " rgb/" + image_name + "\n")

        num += 1
