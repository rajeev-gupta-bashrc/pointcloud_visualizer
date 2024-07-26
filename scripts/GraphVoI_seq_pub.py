#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import pickle
import numpy as np
import os, sys
from collections import OrderedDict
from datetime import datetime
import time

sys.path.insert(0, os.path.dirname(__file__))
from point_cloud_publisher import *

def get_points_from_bin(file_path):
    point_array = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)
    point_array = np.delete(point_array, 3, axis=1)
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    point_cloud = pc2.create_cloud_xyz32(header, point_array)
    return point_cloud



def main():
    rospy.init_node('seq_publisher', anonymous=True)
    rospy.loginfo("Waiting for 3 seconds...")
    # time.sleep(3)
    rospy.loginfo("Done waiting!")
    pub_pc = rospy.Publisher('/point_cloud_k', PointCloud2, queue_size=10)
    pub_bbox = rospy.Publisher('/bounding_box_marker_k', Marker, queue_size=10)
    rate = rospy.Rate(10)
    
    frame_path = '/home/rajeev-gupta/sensyn_ws/src/GD-MAE/data/kitti/testing/velodyne/'
    preds_path = '/home/rajeev-gupta/sensyn_ws/src/GD-MAE/data/kitti/testing/predi_2/'
    # anns_path = '/home/rajeev-gupta/sensyn_ws/src/GD-MAE/data/kitti/testing/label/'
    pred_files = sorted(os.listdir(preds_path))
    # ann_files = os.listdir(preds_path)
    
    for p in range(len(pred_files)):
        rospy.loginfo(f'---------Header--------- \nLoading the {pred_files[p]} frame')
        
        results = []          
        for line in open(preds_path+pred_files[p], 'r'):
            line = line.split()
            results.append(line)
        # print(results)
        # anns = []
        # with open(anns_path+ann_files[p], 'r') as file:
        #     lines = file.read()
        #     for line in lines:
        #         a = line.split()
        #         anns.append(a)
        
        # anns_bbox7s = [a[4:11] for a in anns]
        predi_bbox7s = [[float(a) for a in r[8:15]] for r in results]
        ## predi_bbox7s : dim(3) center(3) rot_y
        ## what we want : center(3) dim(3) rot_y
        # print(predi_bbox7s, 'grgr')
        # for b in predi_bbox7s:
        #     dim = b[:3]
        #     center = b[3:6]
        #     b[:3] = center
        #     b[3:6] = dim
        #     x = b[0]
        #     y = b[1]
        #     z = b[2]
            # b[0] = -z
            # b[1] = -x
            # b[2] = y
        # print(predi_bbox7s)
        
        
        # print(predi_bbox7s)
        # rospy.loginfo(predi_bbox7s)
        # anns_markers = []
        predi_markers = []
        # for i in range(len(anns_bbox7s)):
        #     anns_markers.append(create_bounding_box_marker(get_8_point_bbox(anns_bbox7s[i]), i, [0, 1, 0], namespace='anns_bbox')) #green annotations
        for i in range(len(predi_bbox7s)):
            predi_markers.append(create_bounding_box_marker(get_8_point_bbox(predi_bbox7s[i]), i, [1, 0, 0], namespace='predi_bbox', duration=1)) #red annotations
        
        # scores = get_scores_bbox(pred)
        
        rospy.loginfo('---------Publishing---------')
        bin_file = pred_files[p].replace('txt', 'bin')
        point_cloud = get_points_from_bin(frame_path+bin_file)
        pub_pc.publish(point_cloud)

        ## To pause at a frame: 
        J = 50
        
        ## Publish
        while not rospy.is_shutdown():
            for marker in predi_markers:
                pub_bbox.publish(marker)
            # for marker in anns_markers:
            #     pub_bbox.publish(marker)
            rate.sleep()
            if J != p: 
                break
        else:
            rospy.loginfo('ROSPY is not available')
        
        current_time = rospy.Time.now()
        current_time_sec = current_time.to_sec()
        human_readable_time = datetime.fromtimestamp(current_time_sec)
        formatted_time = human_readable_time.strftime('%Y-%m-%d %H:%M:%S')
        rospy.loginfo(f"At ROSPY time: {formatted_time}\n")
    
    
    
if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    #         Z   
    #         |   Y
    #         |  /
    # RIGHT   | /                     LEFT
    #         |/
    #         *------------>X