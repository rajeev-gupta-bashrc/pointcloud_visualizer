#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


import pickle
import numpy as np
import os
# import torch


# ######### ROS things :)
# # from object_detector import module
# from object_detector.module import call
# call()


def get_8_point_bbox(point7):
    try:
        bbox7 = point7.cpu().numpy()
    except Exception:
        bbox7 = point7
    theta = float(bbox7[6])
    cx, cy, cz = float(bbox7[0]), float(bbox7[1]), float(bbox7[2])
    dx, dy, dz = float(bbox7[3]), float(bbox7[4]), float(bbox7[5])
    # theta = 0
    # print(len(bbox7))
    # print(theta)
    hx, hy, hz = dx / 2, dy / 2, dz / 2
    corners = np.array([
        [hx, hy, hz],
        [hx, hy, -hz],
        [hx, -hy, -hz],
        [hx, -hy, hz],
        [-hx, hy, hz],
        [-hx, hy, -hz],
        [-hx, -hy, -hz],
        [-hx, -hy, hz],
    ])    
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    rotation_matrix = np.array([
        [cos_theta, -sin_theta, 0],
        [sin_theta, cos_theta, 0],
        [0, 0, 1]
    ])
    rotated_corners = np.dot(corners, rotation_matrix.T)
    global_corners = rotated_corners + np.array([cx, cy, cz])
    return global_corners
    
    
def num_pred_bbox(pred):
    return len(pred['box3d_lidar'])

def get_bbox_points_from_pred(pred):
    return pred['box3d_lidar']

def create_bounding_box_marker(bbox_point, id, rgb = None, namespace = "", duration=2):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace
    marker.id = id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD

    edges = [
        # (0, 1), (1, 3), (3, 2), (2, 0),  # Top face
        # (4, 5), (6, 4), (5, 7), (7, 6),  # Bottom face
        # (0, 4), (1, 5), (2, 6), (3, 7)   # Vertical lines
        (0, 1), (1, 2), (2, 3), (3, 0),  # front face YZ
        (4, 5), (5, 6), (6, 7), (7, 4),  # rear face YZ
        (0, 4), (1, 5), (2, 6), (3, 7),  # Horizontal lines X
        (0, 2), (1, 3)                   # Cross in front face
    ]

    for start, end in edges:
        p1 = Point()
        p1.x, p1.y, p1.z = bbox_point[start]
        marker.points.append(p1)
        p2 = Point()
        p2.x, p2.y, p2.z = bbox_point[end]
        marker.points.append(p2)

    marker.pose.orientation = Quaternion()
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    
    marker.scale.x = 0.05  # Line width
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    if rgb is not None and len(rgb) == 3:
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
    _duration = rospy.rostime.Duration(duration)
    marker.lifetime = _duration
    return marker


def get_points_from_pkl(lidar_data):
    point_array = lidar_data['lidars']['points_xyz']
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    point_cloud = pc2.create_cloud_xyz32(header, point_array)
    return point_cloud
    
def get_scores_bbox(pred):
    return pred['scores']
    
def load_pickle(path, format):
    with open(path, format) as file:
        data = pickle.load(file)
    return data

def get_bbox7_from_anns(anns):
    bbox7s = [obj['box'] for obj in anns['objects']]
    return bbox7s


def main():
    rospy.init_node('point_cloud_publisher', anonymous=True)
    pub_pc = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
    pub_bbox = rospy.Publisher('/bounding_box_marker', Marker, queue_size=10)
    score_threshold = 0.50
    rate = rospy.Rate(5)
    
    frame_path = '/home/rajeev-gupta/sensyn_ws/src/GraphRCNN/Ventoy/waymo_data/data/waymo/waymo_processed_data_cp/train/lidar/seq_0_frame_0.pkl'
    preds_path = '/home/rajeev-gupta/sensyn_ws/src/GraphRCNN/work_dirs/waymo_centerpoint_voxelnet_graphrcnn_6epoch_freeze/re_preds_5.pkl'
    annos_path = '/home/rajeev-gupta/sensyn_ws/src/GraphRCNN/Ventoy/waymo_data/data/waymo/waymo_processed_data_cp/val/annos/'
    
    lidar_data = load_pickle(frame_path, 'rb')
    preds = load_pickle(preds_path, 'rb')
    basename = os.path.basename(frame_path)
    pred = preds[basename]
    anns = load_pickle(os.path.join(annos_path, basename), 'rb')
    anns_bbox7s = get_bbox7_from_anns(anns)
    anns_markers = []
    for i in range(len(anns_bbox7s)):
        anns_markers.append(create_bounding_box_marker(get_8_point_bbox(anns_bbox7s[i]), i, [0, 1, 0], namespace='anns_bbox')) #green annotations
    # print('created markers of anns bbox')
    point_cloud = get_points_from_pkl(lidar_data)
    pred_bbox_markers = []
    bbox_points7 = get_bbox_points_from_pred(pred)
    scores = get_scores_bbox(pred)
    
    # for bbox7, score in zip(bbox_points7, scores):
    #     if score>=score_threshold:
    #         bbox8 = get_8_point_bbox(bbox7)
    #         marker = create_bounding_box_marker(bbox8)
    #         pred_bbox_markers.append(marker)

    for i in range(num_pred_bbox(pred)):
        if scores[i]>=score_threshold:
            bbox8 = get_8_point_bbox(bbox_points7[i])
            marker = create_bounding_box_marker(bbox8, i, namespace='pred_bbox') #red predicted boxes
            pred_bbox_markers.append(marker)
    
    pub_pc.publish(point_cloud)
    while not rospy.is_shutdown():
        for marker in pred_bbox_markers:
            pub_bbox.publish(marker)
        for marker in anns_markers:
            pub_bbox.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
