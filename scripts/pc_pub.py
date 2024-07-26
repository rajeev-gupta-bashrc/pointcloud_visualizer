#!/usr/bin/env python
from point_cloud_publisher import *

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import numpy as np

def pc_range2bbox(pc_range):
    x, y, z, X, Y, Z = pc_range
    ox, oy, oz = (x+X)/2, (y+Y)/2, (z+Z)/2
    w, l, h = X-x, Y-y, Z-z
    rot_y = 0
    bbox = get_8_point_bbox([ox, oy, oz, w, l, h, rot_y])
    print(bbox)
    return bbox

if __name__=='__main__':
    rospy.init_node('point_cloud_publisher', anonymous=True)
    pub_pc = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
    pub_bbox = rospy.Publisher('/bounding_box_marker', Marker, queue_size=10)
    
    # points = np.fromfile('/media/rajeev-gupta/Drive250/SENSYN_/CMT/data/nuscenes/samples/LIDAR_TOP/n015-2018-10-08-15-36-50+0800__LIDAR_TOP__1538984252446890.pcd.bin', dtype=np.float32).reshape(-1, 5)[:, :3]
    # points = np.fromfile('/home/rajeev-gupta/sensyn_ws/src/GD-MAE/tools/new_points_kt.bin', dtype=np.float32).reshape(-1, 3)
    # points = np.fromfile('/home/rajeev-gupta/sensyn_ws/src/GD-MAE/data/waymo/segment-13182548552824592684_4160_250_4180_250_with_camera_label/velodyne/000010.bin', dtype=np.float32).reshape(-1, 4)[:, :3]
    # points = np.fromfile('/home/rajeev-gupta/sensyn_ws/src/GD-MAE/tools/pts_rect.bin', dtype=np.float32).reshape(-1, 3)
    # points = np.fromfile('/home/rajeev-gupta/sensyn_ws/src/GD-MAE/data/kitti/testing/velodyne/000007.bin', dtype=np.float32).reshape(-1, 3)
    # points = np.fromfile('/home/rajeev-gupta/sensyn_ws/src/GD-MAE/data/kitti/testing/velodyne/000008.bin', dtype=np.float32).reshape(-1, 4)[:, :3]
    # points = np.fromfile('/home/rajeev-gupta/sensyn_ws/src/GD-MAE/tools/new_points_wo.bin', dtype=np.float32).reshape(-1, 3)
    points = np.fromfile('/media/rajeev-gupta/Drive250/SENSYN_/from_sensyn_ws_src/GraphRCNN/Ventoy/waymo_data/data/waymo/kitti/raw_data/segment-13145971249179441231_1640_000_1660_000_with_camera_label/velodyne/000000.bin', dtype=np.float32).reshape(-1, 5)[:, :3]
    print(points[0])
    # print(points)
    print('points shape ', points.shape)
    for i in range(3):
        print(min(points[:, i]), max(points[:, i]))
        
    point_cloud_range = [
                        # [-40, 0, -3, 40, 70.4, 1],
                        # [-51.2, -51.2, -5.0, 51.2, 51.2, 3.0],
                        # [-51.2, 0, -5.0, 51.2, 51.2, 3.0],
                        [0, -40, -3, 70.4, 40, 2],
                        ]
    
    bbox_7s = [
            [ 10.762794  ,  -3.3861363 ,   0.9047416 ,   1.9963179 ,
          4.20393   ,   1.6638173 ,  -1.5909283 ],
      #  [ 18.872583  ,  -2.7528381 ,   0.84128463,   1.9818243 ,
      #     4.3715434 ,   1.487088  ,  -1.6426747 ],
      #  [  3.7015488 ,  -3.3105464 ,   0.8155584 ,   1.9355491 ,
      #     4.216762  ,   1.5408412 ,  -1.56094   ],
      #  [ 24.562313  ,   2.0099068 ,   0.9787841 ,   1.9487857 ,
      #     4.122313  ,   1.7320704 ,  -2.1083982 ],
      #  [  9.062783  ,   0.17796743,   0.9427309 ,   2.1263313 ,
      #     4.333297  ,   1.7646408 ,  -1.5860733 ],
      #  [ 25.068903  ,  -2.5774844 ,   0.9481651 ,   1.9785252 ,
      #     4.3659043 ,   1.5680995 ,  -1.6778716 ],
      #  [ 28.550085  ,   1.1371177 ,   1.0179018 ,   1.996651  ,
      #     4.3671026 ,   1.7100234 ,  -1.6819236 ],
      #  [  0.9475867 ,  -8.039181  ,   1.1951908 ,   0.8622462 ,
      #     1.0609007 ,   1.8440729 ,   1.5924141 ],
      #  [ 16.900785  ,   0.08290592,   0.8368907 ,   2.0684133 ,
      #     4.2241645 ,   1.4375796 ,  -1.6372606 ],
      #  [ 15.9503355 , -10.193804  ,   1.2900901 ,   0.84733206,
      #     1.1599916 ,   1.8130002 ,  -1.7014965 ],
      #  [ 67.142166  ,   5.7147527 ,   1.0576816 ,   2.0403342 ,
      #     4.4339466 ,   1.5903757 ,  -1.6876812 ],
      #  [ 31.940933  ,  -1.6648728 ,   0.9184248 ,   2.0580337 ,
      #     4.786276  ,   1.5215929 ,  -1.6714069 ],
      #  [ 63.93003   ,  -9.488409  ,   1.4751985 ,   0.9488394 ,
      #     1.0552707 ,   1.7840453 ,   1.5279452 ]
    ]
    # filter points
    # x, y, z = point_cloud_range[1][:3]
    # X, Y, Z = point_cloud_range[1][3:]
    # mask = (points[:, 0] >= x) & (points[:, 0] <= X) & \
    #        (points[:, 1] >= y) & (points[:, 1] <= Y) & \
    #        (points[:, 2] >= z) & (points[:, 2] <= Z)
    # points = points[mask]
    
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    point_cloud = pc2.create_cloud_xyz32(header, points)
    
    markers = []
    for i, pc_range in enumerate(point_cloud_range):
        bbox = pc_range2bbox(pc_range)
        marker = create_bounding_box_marker(bbox_point=bbox, id=i)
        markers.append(marker)
        
    for i, bbox in enumerate(bbox_7s):
        bbox = get_8_point_bbox(bbox)
        marker = create_bounding_box_marker(bbox_point=bbox, id=i, namespace='pred_bbox')
        markers.append(marker)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub_pc.publish(point_cloud)
        for marker in markers: pub_bbox.publish(marker)
        rate.sleep()
    
    
'''
    
for Nuscenes:

                            Front


                        Z   
                        |   Y
                        |  /
                LEFT    | /                     RIGHT
                        |/
                        *------------>X
        
                    Back
        
        
'''