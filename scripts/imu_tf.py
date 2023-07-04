import rosbag
import numpy as np

bag_file = '/path/to/your/bag/file.bag'
accelerometer_topic = '/accelerometer'
groundtruth_topic = '/groundtruth'
output_dir = '/path/to/output/directory/'
output_accel_file = output_dir + 'accelerometer.txt'
output_gt_file = output_dir + 'groundtruth.txt'

# Open the bag file
bag = rosbag.Bag(bag_file)

# Initialize lists to store data
accel_data = []
gt_data = []

# Read messages from the bag file
for topic, msg, t in bag.read_messages(topics=[accelerometer_topic, groundtruth_topic]):
    if topic == accelerometer_topic:
        # Extract accelerometer data
        timestamp = msg.header.stamp.to_sec()
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z
        accel_data.append([timestamp, accel_x, accel_y, accel_z])
    elif topic == groundtruth_topic:
        # Extract ground truth data
        timestamp = msg.header.stamp.to_sec()
        position_x = msg.pose.position.x
        position_y = msg.pose.position.y
        position_z = msg.pose.position.z
        # Add more code here to extract orientation if needed
        gt_data.append([timestamp, position_x, position_y, position_z])

# Save the data to files
np.savetxt(output_accel_file, accel_data, fmt='%.6f %.6f %.6f %.6f')
np.savetxt(output_gt_file, gt_data, fmt='%.6f %.6f %.6f %.6f')

# Close the bag file
bag.close()
