'''
以下这些代码只是示例，如果想制作TUM数据集groundtruth.txt文件
你可能还需要知道绝对位姿的话题，笔者并不确定以下这些代码是否行得
通，当然你可以通过以下这些代码去进行修改，从而制作完整的TUM数据集
以下这些代码只做参考！！！真实情况需要自己琢磨思考并完成
注意事项：TUM官方数据集给的imu话题可能导不出groundtruth.txt文件
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
bag_file = "/home/zhao/tum/rgbd_dataset_freiburg1_xyz.bag"

# 相机彩色图像话题
image_topic = "/camera/rgb/image_color"

# 绝对位姿话题
pose_topic = "/pose_topic"

groundtruth_file = "/home/zhao/tum/groundtruth.txt"

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
                # 这是一个示例,在这里添加视觉里程计算法来估计相机位姿

                # 转换为灰度图像
                prev_image_gray = cv2.cvtColor(prev_image, cv2.COLOR_BGR2GRAY)
                cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

                # 检查图像尺寸是否匹配
                if prev_image_gray.shape == cv_image_gray.shape:
                    # 计算光流
                    optical_flow = cv2.calcOpticalFlowFarneback(prev_image_gray, cv_image_gray, None, 0.5, 3, 15, 3, 5,
                                                                1.2, 0)

                    # 计算位姿变化
                    delta_tx = np.mean(optical_flow[..., 0])
                    delta_ty = np.mean(optical_flow[..., 1])
                    delta_tz = 0.0  # 假设在平面运动中，z轴不发生变化

                    # 更新位姿信息
                    result = kalman_filter.update([delta_tx, delta_ty, delta_tz])
                    if result is not None:
                        tx, ty, tz, roll, pitch, yaw = result

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
                    # 处理图像尺寸不匹配的情况
                    print("Error: Image sizes do not match.")

            else:
                # 如果是第一帧图像，仅更新先前图像
                prev_image = cv_image

        elif topic == pose_topic:
            # 获取相机的绝对位姿信息
            if hasattr(msg, 'pose') and hasattr(msg.pose, 'position') and hasattr(msg.pose, 'orientation'):
                tx = msg.pose.position.x
                ty = msg.pose.position.y
                tz = msg.pose.position.z
                qx = msg.pose.orientation.x
                qy = msg.pose.orientation.y
                qz = msg.pose.orientation.z
                qw = msg.pose.orientation.w

                # 将四元数转换为欧拉角
                roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

                # 更新Kalman滤波器状态
                result = kalman_filter.update([tx, ty, tz])
                if result is not None:
                    # 保存位姿信息到groundtruth.txt文件
                    pose_line = "{} {} {} {} {} {} {}\n".format(t.to_sec(), tx, ty, tz, roll, pitch, yaw)
                    gt_file.write(pose_line)

                    # 输出已写入的位姿信息
                    print(pose_line)

                    # 更新先前位姿
                    prev_tx, prev_ty, prev_tz = tx, ty, tz

    bag.close()
