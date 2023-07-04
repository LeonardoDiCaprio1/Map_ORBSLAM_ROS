import os
import rosbag
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rgb_path = '/path/to/save/bag/rgb/'  # 存储提取的彩色图像的绝对路径
depth_path = '/path/to/save/bag/depth/'  # 存储提取的深度图像的绝对路径
bridge = CvBridge()
num = 1

with rosbag.Bag('/path/to/image.bag', 'r') as bag:
    for topic, msg, t in bag.read_messages():
        if topic == "/camera/depth/image_raw":
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
                image_name = str(num) + '.png'
                cv2.imwrite(os.path.join(depth_path, image_name), cv_image)
                print(os.path.join(depth_path, image_name))
            except CvBridgeError as e:
                print(e)
        elif topic == "/camera/rgb/image_raw":
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                image_name = str(num) + '.png'
                cv2.imwrite(os.path.join(rgb_path, image_name), cv_image)
                print(os.path.join(rgb_path, image_name))
            except CvBridgeError as e:
                print(e)
        num += 1
