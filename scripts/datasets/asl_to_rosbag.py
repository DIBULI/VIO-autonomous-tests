import os
import rosbag
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import yaml
import rospy
import pandas as pd
import argparse
import zipfile
import tempfile
import shutil

def load_yaml(filename):
    with open(filename, 'r') as file:
        return yaml.safe_load(file)

def read_csv(filepath):
    df = pd.read_csv(filepath)
    df.columns = df.columns.str.strip()
    return df

def find_sensor_folders(dataset_path, sensor_type):
    sensor_folders = []
    for folder in os.listdir(dataset_path):
        folder_path = os.path.join(dataset_path, folder)
        if os.path.isdir(folder_path):
            sensor_yaml = os.path.join(folder_path, 'sensor.yaml')
            if os.path.exists(sensor_yaml):
                sensor_info = load_yaml(sensor_yaml)
                if sensor_info.get('sensor_type') == sensor_type:
                    sensor_folders.append(folder)
    return sensor_folders

def process_camera_data(bag, dataset_path, camera_folders, bridge, verbose):
    for cam in camera_folders:
        cam_path = os.path.join(dataset_path, cam)
        cam_data = read_csv(os.path.join(cam_path, 'data.csv'))
        cam_info = load_yaml(os.path.join(cam_path, 'sensor.yaml'))
        
        if verbose:
            print(f"Processing {cam} data from {cam_path}...")

        for idx, row in cam_data.iterrows():
            timestamp_ns = row.iloc[0]
            timestamp = rospy.Time.from_sec(int(timestamp_ns) * 1e-9)
            image_path = os.path.join(cam_path, 'data', row['filename'])
            if verbose:
                print(f"Reading image from {image_path}")
            image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            image_msg = bridge.cv2_to_imgmsg(image, encoding="mono8")
            image_msg.header.stamp = timestamp
            image_msg.header.frame_id = cam
            bag.write(f'/{cam}/image_raw', image_msg, timestamp)

def process_imu_data(bag, dataset_path, imu_folders, verbose):
    for imu in imu_folders:
        imu_path = os.path.join(dataset_path, imu)
        imu_data = read_csv(os.path.join(imu_path, 'data.csv'))
        imu_info = load_yaml(os.path.join(imu_path, 'sensor.yaml'))
        
        if verbose:
            print(f"Processing {imu} data from {imu_path}...")

        for idx, row in imu_data.iterrows():
            timestamp_ns = row.iloc[0]
            timestamp = rospy.Time.from_sec(int(timestamp_ns) * 1e-9)
            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            imu_msg.header.frame_id = imu
            imu_msg.angular_velocity.x = row['w_RS_S_x [rad s^-1]']
            imu_msg.angular_velocity.y = row['w_RS_S_y [rad s^-1]']
            imu_msg.angular_velocity.z = row['w_RS_S_z [rad s^-1]']
            imu_msg.linear_acceleration.x = row['a_RS_S_x [m s^-2]']
            imu_msg.linear_acceleration.y = row['a_RS_S_y [m s^-2]']
            imu_msg.linear_acceleration.z = row['a_RS_S_z [m s^-2]']
            bag.write(f'/{imu}', imu_msg, timestamp)

def process_leica_data(bag, dataset_path, leica_folders, verbose):
    for leica in leica_folders:
        leica_path = os.path.join(dataset_path, leica)
        leica_data = read_csv(os.path.join(leica_path, 'data.csv'))
        
        if verbose:
            print(f"Processing {leica} data from {leica_path}...")

        for idx, row in leica_data.iterrows():
            timestamp_ns = row.iloc[0]
            timestamp = rospy.Time.from_sec(int(timestamp_ns) * 1e-9)
            point_msg = PointStamped()
            point_msg.header.stamp = timestamp
            point_msg.header.frame_id = leica
            point_msg.point.x = row['p_RS_R_x [m]']
            point_msg.point.y = row['p_RS_R_y [m]']
            point_msg.point.z = row['p_RS_R_z [m]']
            bag.write(f'/{leica}', point_msg, timestamp)

def process_groundtruth_data(bag, dataset_path, groundtruth_folders, verbose):
    for gt in groundtruth_folders:
        gt_path = os.path.join(dataset_path, gt)
        gt_data = read_csv(os.path.join(gt_path, 'data.csv'))
        
        if verbose:
            print(f"Processing {gt} data from {gt_path}...")

        for idx, row in gt_data.iterrows():
            timestamp_ns = row.iloc[0]
            timestamp = rospy.Time.from_sec(int(timestamp_ns) * 1e-9)
            odom_msg = Odometry()
            odom_msg.header.stamp = timestamp
            odom_msg.header.frame_id = gt
            odom_msg.pose.pose.position.x = row['p_RS_R_x [m]']
            odom_msg.pose.pose.position.y = row['p_RS_R_y [m]']
            odom_msg.pose.pose.position.z = row['p_RS_R_z [m]']
            odom_msg.pose.pose.orientation.w = row['q_RS_w []']
            odom_msg.pose.pose.orientation.x = row['q_RS_x []']
            odom_msg.pose.pose.orientation.y = row['q_RS_y []']
            odom_msg.pose.pose.orientation.z = row['q_RS_z []']
            bag.write(f'/{gt}', odom_msg, timestamp)

def process_vicon_data(bag, dataset_path, vicon_folders, verbose):
    for vicon in vicon_folders:
        vicon_path = os.path.join(dataset_path, vicon)
        vicon_data = read_csv(os.path.join(vicon_path, 'data.csv'))
        
        if verbose:
            print(f"Processing {vicon} data from {vicon_path}...")

        for idx, row in vicon_data.iterrows():
            timestamp_ns = row.iloc[0]
            timestamp = rospy.Time.from_sec(int(timestamp_ns) * 1e-9)
            odom_msg = Odometry()
            odom_msg.header.stamp = timestamp
            odom_msg.header.frame_id = vicon
            odom_msg.pose.pose.position.x = row['p_RS_R_x [m]']
            odom_msg.pose.pose.position.y = row['p_RS_R_y [m]']
            odom_msg.pose.pose.position.z = row['p_RS_R_z [m]']
            odom_msg.pose.pose.orientation.w = row['q_RS_w []']
            odom_msg.pose.pose.orientation.x = row['q_RS_x []']
            odom_msg.pose.pose.orientation.y = row['q_RS_y []']
            odom_msg.pose.pose.orientation.z = row['q_RS_z []']
            bag.write(f'/{vicon}', odom_msg, timestamp)

def create_bag(output_bag, dataset_path, verbose=False):
    output_dir = os.path.abspath(os.path.dirname(output_bag))
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    output_bag = os.path.join(output_dir, os.path.basename(output_bag))

    bridge = CvBridge()
    bag = rosbag.Bag(output_bag, 'w')
    
    try:
        camera_folders = find_sensor_folders(dataset_path, 'camera')
        imu_folders = find_sensor_folders(dataset_path, 'imu')
        leica_folders = find_sensor_folders(dataset_path, 'position')
        groundtruth_folders = find_sensor_folders(dataset_path, 'visual-inertial')
        vicon_folders = find_sensor_folders(dataset_path, 'pose')
        
        if verbose:
            print(f"Found camera folders: {camera_folders}")
            print(f"Found IMU folders: {imu_folders}")
            print(f"Found Leica folders: {leica_folders}")
            print(f"Found ground truth folders: {groundtruth_folders}")
            print(f"Found Vicon folders: {vicon_folders}")

        process_camera_data(bag, dataset_path, camera_folders, bridge, verbose)
        process_imu_data(bag, dataset_path, imu_folders, verbose)
        process_leica_data(bag, dataset_path, leica_folders, verbose)
        process_groundtruth_data(bag, dataset_path, groundtruth_folders, verbose)
        process_vicon_data(bag, dataset_path, vicon_folders, verbose)

    finally:
        bag.close()
        if verbose:
            print(f"Finished writing to {output_bag}")

def handle_zip_input(zip_path, verbose=False):
    temp_dir = tempfile.mkdtemp()
    if verbose:
        print(f"Extracting {zip_path} to temporary directory {temp_dir}...")
    with zipfile.ZipFile(zip_path, 'r') as zip_ref:
        zip_ref.extractall(temp_dir)
    extracted_dir = os.path.join(temp_dir, "mav0")
    if verbose:
        print(f"Extracted to {extracted_dir}")
    return extracted_dir

def clean_up_temp_dir(temp_dir, verbose=False):
    if verbose:
        print(f"Cleaning up temporary directory {temp_dir}...")
    shutil.rmtree(temp_dir)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ASL dataset to ROS bag")
    parser.add_argument('--path', required=True, help="Path to the dataset directory or ZIP file")
    parser.add_argument('--output', required=True, help="Output ROS bag file")
    parser.add_argument('-v', '--verbose', action='store_true', help="Output the progress")

    args = parser.parse_args()

    if zipfile.is_zipfile(args.path):
        temp_dir = handle_zip_input(args.path, args.verbose)
        create_bag(args.output, temp_dir, args.verbose)
        clean_up_temp_dir(temp_dir, args.verbose)
    else:
        create_bag(args.output, args.path, args.verbose)
