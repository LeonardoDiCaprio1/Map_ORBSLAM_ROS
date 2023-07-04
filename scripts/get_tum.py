import numpy
import rosbag
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
rgb_path = '/path/to/save/bag/rgb/'
depth_path = '/path/to/save/bag/depth/'
bridge = CvBridge()
with rosbag.Bag('/path/to/image.bag', 'r') as bag:
	num = 1
for topic,msg,t in bag.read_messages():
        
        if topic == "/camera/depth/image_raw": 
            cv_image = bridge.imgmsg_to_cv2(msg, '32FC1')
            cv_image = cv_image * 255 
            # timestr = "%.8f" %  msg.header.stamp.to_sec() # 时间戳命名
            # image_name = timestr + '.png'# an extension is necessary
            image_name = str(num) + '.png'# 编号命名
            cv2.imwrite(depth_path + image_name, cv_image)  
            # 实际应用可直接保存为 numpy array
            # np.save(depth_path + image_name, cv_image)  
            print(depth_path + image_name)
        if topic == "/camera/rgb/image_raw": 
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            timestr = "%.8f" %  msg.header.stamp.to_sec()
            image_name = str(num) + '.png'
            cv2.imwrite(rgb_path + image_name, cv_image)
            num += 1
