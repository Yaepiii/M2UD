#!/usr/bin/env python

import rospy
import rosbag
from tf2_msgs.msg import TFMessage
import sys
import os

'''
 * Copyright (c) 2024, Shenyang Institute of Automation, Chinese Academy of Sciences
 *
 * Authors: Yanpeng Jia
 * Contact: yaepiii@126.com
 * 
 * Usage: Read bag file, fliter and remap topic
 *
'''
def filter_remap_remove_tf(in_bag, out_bag):
    topics_to_keep = ['/RTK/data_raw', '/imu/data', '/imu/mag', '/velodyne_points', '/camera/color/image_raw', '/camera/depth/image_rect_raw', '/livox/imu', '/livox/lidar', '/wit/imu', '/wit/mag']
    topic_remap = {'/camera/color/image_raw': '/camera/color', '/camera/depth/image_rect_raw': '/camera/depth', '/RTK/data_raw': '/RTK/data'}

    with rosbag.Bag(out_bag, 'w') as outbag_handle:
        for topic, msg, t in rosbag.Bag(in_bag).read_messages():
            if topic in topics_to_keep:
                print("Dealing bag file: %-30s time: %-10f topic: %s " %(in_bag, t.to_sec(), topic))
                if topic in topic_remap:
                    outbag_handle.write(topic_remap[topic], msg, t)
                else:
                    outbag_handle.write(topic, msg, t)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: deal_bag.py <deal bag root>")
        sys.exit(1)
    file_root = sys.argv[1]
    for file_name in os.listdir(file_root + '/raw/'):
        if os.path.splitext(file_name)[1] == '.bag':
            in_bag = file_root + '/raw/' + file_name
            out_bag = file_root + file_name
            print("Dealing bag file: %s" %in_bag)
            filter_remap_remove_tf(in_bag, out_bag)
            print("Deal finish!!!!!!!!!!!!!!!!!!!!")
    #if len(sys.argv) < 3:
    #    print("Usage: deal_bag.py <input_bag> <output_bag>")
    #    sys.exit(1)
    #inbag = sys.argv[1]
    #outbag = sys.argv[2]

    #filter_remap_remove_tf(inbag, outbag)

