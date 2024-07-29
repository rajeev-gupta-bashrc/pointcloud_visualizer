# Point Cloud Visualizer

This package provides visualization capabilities for point clouds and bounding box markers in a ROS environment. The `pc_pub.py` script loads point cloud data from a `.bin` file and publishes it over ROS. To visualize a bounding box marker, append the bbox array to the `bbox_7s` array in the following format:

```
[centre_X, centre_Y, centre_Z, length, breadth, height, Yaw]
```

## Installation

Clone the package to your workspace:

```bash
cd ~
cd your_ws/src
git clone https://github.com/rajeev-gupta-bashrc/pointcloud_visualizer.git
cd ..
catkin_make
```

## Usage

1. Launch RViz:

```bash
roslaunch pointcloud_visualizer rviz.launch
```

2. Run the publisher node:

```bash
rosrun pointcloud_visualizer pc_pub
```

This will start the point cloud publisher and allow you to visualize your point cloud data along with any bounding box markers you've defined in RViz.

Remember to source your workspace before running these commands if you haven't added it to your `.bashrc`:

```bash
source ~/your_ws/devel/setup.bash
```

You can now use RViz to view and interact with your published point cloud and bounding box markers.