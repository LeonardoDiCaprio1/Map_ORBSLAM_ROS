'''
注意事项：作者本人没有使用过这些代码
这些代码只是样例！！！
如果你录制过imu话题和tf话题
你可以可依据这些代码进行更改并修复一些可能存在的bug
如果能成功请在作者本人的博客回复作者，感谢！
'''

import rosbag
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from filterpy.kalman import KalmanFilter

# 设置文件路径和话题名称
bag_file = "/path/to/bag_file.bag"

#imu话题
image_topic = "/camera/image"

#tf话题
pose_topic = "/camera/pose"           

groundtruth_file = "/path/to/groundtruth.txt" 

# 创建CvBridge对象
bridge = CvBridge()

# 创建groundtruth.txt文件并写入位姿信息
with open(groundtruth_file, 'w') as gt_file:
    # 初始化Kalman滤波器
    kalman_filter = KalmanFilter(dim_x=6, dim_z=3)
    # 初始状态估计
    kalman_filter.x = np.array([0, 0, 0, 0, 0, 0])  
    # 状态转移矩阵
    kalman_filter.F = np.eye(6)  
    # 测量矩阵
    kalman_filter.H = np.eye(3, 6) 
    # 初始协方差估计
    kalman_filter.P *= 1000  
    # 测量噪声协方差
    kalman_filter.R = np.eye(3) * 0.01  
    # 过程噪声协方差
    kalman_filter.Q = np.eye(6) * 0.01  

    # 读取ROS bag文件
    bag = rosbag.Bag(bag_file)

    # 初始化先前图像和位姿
    prev_image = None
    prev_tx, prev_ty, prev_tz = 0.0, 0.0, 0.0

    for topic, msg, t in bag.read_messages(topics=[image_topic, pose_topic]):
        if topic == image_topic:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if prev_image is not None:
                # 这是一个简化示例
                # 在这里添加视觉里程计算法来估计相机位姿
                
                # 假设使用光流法来估计位姿变化
                optical_flow = cv2.calcOpticalFlowFarneback(prev_image, cv_image, None, 0.5, 3, 15, 3, 5, 1.2, 0)

                # 计算位姿变化
                delta_tx = np.mean(optical_flow[..., 0])
                delta_ty = np.mean(optical_flow[..., 1])
                delta_tz = 0.0  # 假设在平面运动中，z轴不发生变化

                # 更新位姿信息
                tx, ty, tz, roll, pitch, yaw = kalman_filter.update([delta_tx, delta_ty, delta_tz])

                # 将变化量与先前位姿相加得到绝对位姿
                tx += prev_tx
                ty += prev_ty
                tz += prev_tz

                # 保存位姿信息到groundtruth.txt文件
                pose_line = "{} {} {} {} {} {} {}\n".format(t.to_sec(), tx, ty, tz, roll, pitch, yaw)
                gt_file.write(pose_line)

                # 输出已写入的位姿信息
                print(pose_line)

                # 更新先前位姿和图像
                prev_tx, prev_ty, prev_tz = tx, ty, tz
                prev_image = cv_image

            else:
                # 如果是第一帧图像，仅更新先前图像
                prev_image = cv_image

        elif topic == pose_topic:
            # 获取相机的绝对位姿信息
            pose = msg.pose

            # 获取位姿的平移和旋转信息
            tx = pose.position.x
            ty = pose.position.y
            tz = pose.position.z
            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w

            # 将四元数转换为欧拉角
            roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

            # 更新Kalman滤波器状态
            kalman_filter.update([tx, ty, tz])

            # 保存位姿信息到groundtruth.txt文件
            pose_line = "{} {} {} {} {} {} {}\n".format(t.to_sec(), tx, ty, tz, roll, pitch, yaw)
            gt_file.write(pose_line)

            # 输出已写入的位姿信息
            print(pose_line)

            # 更新先前位姿
            prev_tx, prev_ty, prev_tz = tx, ty, tz

    bag.close()
