import os
import cv2
import rosbag
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 定义文件路径
script_dir = os.getcwd()
rgb_path = os.path.join(script_dir, 'rgb')
depth_path = os.path.join(script_dir, 'depth')

# 创建 CvBridge 实例
bridge = CvBridge()

# 定义常量
DEPTH_ENCODING = '32FC1'
DEPTH_MIN = 0.0
DEPTH_MAX = 8.5

# 无符号16位整数的最大值
DEPTH_MAX_UINT16 = np.uint16(2**16 - 1)

num = 1
rgb_width = None
rgb_height = None

# 确保文件夹存在
if not os.path.exists(rgb_path):
    os.makedirs(rgb_path)
if not os.path.exists(depth_path):
    os.makedirs(depth_path)

bag_file_path = 'rgbd_dataset_freiburg1_rpy.bag'  # 替换为您的TUM数据集路径

with rosbag.Bag(bag_file_path, 'r') as bag:
    with open(os.path.join(script_dir, 'depth.txt'), 'w') as depth_time_file, open(os.path.join(script_dir, 'rgb.txt'), 'w') as rgb_time_file:
        for topic, msg, t in bag.read_messages():
            if topic == "/camera/depth/image":
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding=DEPTH_ENCODING)

                # 清理和修复深度图像
                cv_image[np.isnan(cv_image)] = DEPTH_MIN  # 将NaN值替换为最小深度值
                cv_image[cv_image < DEPTH_MIN] = DEPTH_MIN  # 将小于最小深度值的值替换为最小深度值
                cv_image[cv_image > DEPTH_MAX] = DEPTH_MAX  # 将大于最大深度值的值替换为最大深度值

                # 将深度图像范围映射到无符号16位整数的范围
                cv_image_normalized = ((cv_image - DEPTH_MIN) / (DEPTH_MAX - DEPTH_MIN) * DEPTH_MAX_UINT16).astype(np.uint16)

                # 获取深度图像尺寸
                depth_height, depth_width = cv_image_normalized.shape

                # 保存深度图像
                image_name = str(num) + '.png'
                cv2.imwrite(os.path.join(depth_path, image_name), cv_image_normalized)

                # 写入深度图像的时间戳
                timestr = "%.8f" % msg.header.stamp.to_sec()
                depth_time_file.write(timestr + " depth/" + image_name + "\n")

            if topic == "/camera/rgb/image_color":
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                rgb_height, rgb_width, _ = cv_image.shape

                # 保存彩色图像
                image_name = str(num) + '.png'
                cv2.imwrite(os.path.join(rgb_path, image_name), cv_image)

                # 写入彩色图像的时间戳
                timestr = "%.8f" % msg.header.stamp.to_sec()
                rgb_time_file.write(timestr + " rgb/" + image_name + "\n")

            num += 1
