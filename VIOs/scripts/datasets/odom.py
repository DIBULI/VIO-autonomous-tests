#!/usr/bin/env python

import rospy
import csv
import rosbag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

def csv_to_rosbag(csv_file, bag_file):
    bag = rosbag.Bag(bag_file, 'w')

    try:
        with open(csv_file, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                odom = Odometry()

                # Convert nanoseconds to seconds and nanoseconds
                timestamp_ns = int(row['#timestamp [ns]'])
                timestamp_sec = timestamp_ns // 1e9
                timestamp_nsec = timestamp_ns % 1e9

                odom.header.stamp = rospy.Time(secs=int(timestamp_sec), nsecs=int(timestamp_nsec))
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"

                odom.pose.pose.position = Point(float(row['p_RS_R_x [m]']),
                                                float(row['p_RS_R_y [m]']),
                                                float(row['p_RS_R_z [m]']))
                odom.pose.pose.orientation = Quaternion(float(row['q_RS_x []']),
                                                        float(row['q_RS_y []']),
                                                        float(row['q_RS_z []']),
                                                        float(row['q_RS_w []']))

                bag.write('/odom', odom, odom.header.stamp)
    finally:
        bag.close()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Convert CSV to ROS bag.')
    parser.add_argument('csv_file', help='Input CSV file with odometry data.')
    parser.add_argument('bag_file', help='Output ROS bag file.')
    args = parser.parse_args()

    rospy.init_node('csv_to_rosbag')
    csv_to_rosbag(args.csv_file, args.bag_file)

