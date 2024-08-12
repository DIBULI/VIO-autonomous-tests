#!/usr/bin/env python3

import rosbag
import sys

if len(sys.argv) != 3:
    print("Usage: python calculate_frequency.py <bag_file> <topic_name>")
    sys.exit(1)

bag_file = sys.argv[1]
topic_name = sys.argv[2]

timestamps = []

with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        timestamps.append(t.to_sec())

if len(timestamps) > 1:
    deltas = [j - i for i, j in zip(timestamps[:-1], timestamps[1:])]
    average_delta = sum(deltas) / len(deltas)
    frequency = 1.0 / average_delta
    print(f"{frequency:.2f}")
else:
    print(f"-1")