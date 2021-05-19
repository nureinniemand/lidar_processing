# A Simple Readme
A ros based library for lidar data processing. The package is splitted into 2 parts:

1. logic: data structure and algorithm without ROS dependency.
2. ros: ros nodes and data convertors act like bridges between ROS and logic code.

## Planned functions
- [x] Internal point cloud container for convienient data processing
- [x] SD point cloud segementation method by D. Streller & K. Dietmayer
- [ ] Grid map structure for history of static environment model
- [ ] Grid map based dynamic classification of point cloud
- [ ] 3D bounding box generation from segmented point cloud
- [ ] 3D bounding box tracking module

## How to compile

>``catkin config --source-space src/``

>``catkin build``

