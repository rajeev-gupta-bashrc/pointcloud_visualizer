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

sys.path.insert(0, os.path.dirname(__file__))
from point_cloud_publisher import *

def sort_frame_names(frame_names):
    frame_dict = {}
    for fname in frame_names:
        tmp = fname
        start = -1
        while(fname[start]!='_'):
            start-=1
        start+=1
        fname = fname[start:-4]
        try:
            index = int(fname)
        except Exception as e:
            raise e
        frame_dict[index] = tmp
    # print(frame_dict)
    sorted_dict = OrderedDict(sorted(frame_dict.items()))
    # print(sorted_dict)
    return list(sorted_dict.values())



def main():
    rospy.init_node('seq_publisher', anonymous=True)
    pub_pc = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
    pub_bbox = rospy.Publisher('/bounding_box_marker', Marker, queue_size=10)
    score_threshold = 0.20
    rate = rospy.Rate(5)
    
    frame_path = '/home/rajeev-gupta/sensyn_ws/src/GraphRCNN/Ventoy/waymo_data/data/waymo/waymo_processed_data_cp/train/lidar/'
    preds_path = '/home/rajeev-gupta/sensyn_ws/src/GraphRCNN/work_dirs/waymo_centerpoint_voxelnet_graphrcnn_6epoch_freeze/re_preds_5.pkl'
    annos_path = '/home/rajeev-gupta/sensyn_ws/src/GraphRCNN/Ventoy/waymo_data/data/waymo/waymo_processed_data_cp/val/annos/'
    
    preds = load_pickle(preds_path, 'rb')
    all_preds = list(preds.keys())
    all_preds = sort_frame_names(all_preds)
    # print(all_preds)
    
    for p in range(len(all_preds)):
        rospy.loginfo(f'---------Header--------- \nLoading the {all_preds[p]} frame')
        
        lidar_data = load_pickle(os.path.join(frame_path, all_preds[p]), 'rb')
        anns = load_pickle(os.path.join(annos_path, all_preds[p]), 'rb')
        pred = preds[all_preds[p]]
        anns_bbox7s = get_bbox7_from_anns(anns)
        anns_markers = []
        for i in range(len(anns_bbox7s)):
            anns_markers.append(create_bounding_box_marker(get_8_point_bbox(anns_bbox7s[i]), i, [0, 1, 0], namespace='anns_bbox')) #green annotations
        point_cloud = get_points_from_pkl(lidar_data)
        pred_bbox_markers = []
        bbox_points7 = get_bbox_points_from_pred(pred)
        scores = get_scores_bbox(pred)

        for i in range(num_pred_bbox(pred)):
            if scores[i]>=score_threshold:
                bbox8 = get_8_point_bbox(bbox_points7[i])
                marker = create_bounding_box_marker(bbox8, i, namespace='pred_bbox') #red predicted boxes
                pred_bbox_markers.append(marker)
        rospy.loginfo('---------Publishing---------')
        pub_pc.publish(point_cloud)
        if not rospy.is_shutdown():
            for marker in pred_bbox_markers:
                pub_bbox.publish(marker)
            for marker in anns_markers:
                pub_bbox.publish(marker)
            rate.sleep()
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
