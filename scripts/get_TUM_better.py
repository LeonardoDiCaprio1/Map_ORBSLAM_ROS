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
    with open('/home/zhao/liu/script/depth.txt', 'w') as depth_time_file, open('/home/zhao/liu/script/rgb.txt', 'w') as rgb_time_file:
        for topic, msg, t in bag.read_messages():
            if topic == "/camera/depth/image_raw":
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

                # 将深度图像转换为32位浮点数（CV_32F）类型
                cv_image_32f = cv_image.astype(np.float32)

                # 将深度图像范围归一化到0-1之间
                cv_image_normalized = cv2.normalize(cv_image_32f, None, 0.0, 1.0, cv2.NORM_MINMAX)

                # 将深度图像扩展到0-65535范围，并转换为16位无符号整数（CV_16U）
                cv_image_mono16 = (cv_image_normalized * 65535).astype(np.uint16)

                # 获取深度图像尺寸
                depth_height, depth_width = cv_image_mono16.shape

                if rgb_width is not None and rgb_height is not None:
                    # 调整深度图像尺寸为与彩色图像相同
                    cv_image_mono16 = cv2.resize(cv_image_mono16, (rgb_width, rgb_height), interpolation=cv2.INTER_LINEAR)

                # 保存深度图像
                image_name = str(num) + '.png'
                cv2.imwrite(depth_path + image_name, cv_image_mono16)
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

