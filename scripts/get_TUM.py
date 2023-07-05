import numpy as np
import rosbag
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rgb_path = '/home/zhao/liu/script/rgb/'
depth_path = '/home/zhao/liu/script/depth/'
bridge = CvBridge()

num = 1
rgb_width = None
rgb_height = None

with rosbag.Bag('/home/zhao/liu/script/image.bag', 'r') as bag:
    with open('depth.txt', 'w') as depth_time_file, open('rgb.txt', 'w') as rgb_time_file:
        for topic, msg, t in bag.read_messages():
            if topic == "/camera/depth/image_raw":
                cv_image = bridge.imgmsg_to_cv2(msg, '32FC1')
                cv_image = cv_image * 255  
                cv_image = cv_image.astype(np.uint16)

                # 获取深度图像尺寸
                depth_height, depth_width = cv_image.shape

                if rgb_width is not None and rgb_height is not None:
                    # 调整深度图像尺寸为与彩色图像相同
                    cv_image = cv2.resize(cv_image, (rgb_width, rgb_height), interpolation=cv2.INTER_LINEAR)

                image_name = str(num) + '.png'
                cv2.imwrite(depth_path + image_name, cv_image)  
                print(depth_path + image_name)

                timestr = "%.8f" % msg.header.stamp.to_sec()
                depth_time_file.write(timestr + " depth/" + image_name + "\n")

            if topic == "/camera/rgb/image_raw": 
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                rgb_height, rgb_width, _ = cv_image.shape
                image_name = str(num) + '.png'
                cv2.imwrite(rgb_path + image_name, cv_image)

                timestr = "%.8f" % msg.header.stamp.to_sec()
                rgb_time_file.write(timestr + " rgb/" + image_name + "\n")

            num += 1
