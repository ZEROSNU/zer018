# lane_vision

Lane vision package folder for SNU ZERO Project
Author: DongWan Kim, SeongGyun Kim

## How to use
git clone and catkin make project

```
cd catkin_ws/src
git clone https://github.com/numpee/lane_vision
cd ..
catkin_make
```

### NODES

There are currently 3 camera nodes: camera_node_collect_data.py, camera_node_raw.py and camera_node.py

camera_node.py is meant for real time processing

camera_node_collect_data.py collects data at 10FPS


### LINE TRACING

```
python process_yellow.py
```


### UPDATES

5.2:
- Updated homography in camera node
- Create and Update lane_detect.py, which subscribes to /warped_image and outputs /lane_map and /waypoints
- Create a fake camera node to simulate driving in a road and debug LiDAR + vision system


### TODO
- Add support for white lane detection, just in case
  --> Varying confidence levels if white lane detection is used.
- Add support for Stop line and parking
- Create traffic sign detection
