#include "pointcloud_segmentation/pointcloud_segmentation.h"

int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "pointcloud_segementation_node");
    lidar_processing::PointCloudSegementation main_processor;
    main_processor.init();
    main_processor.run();
    return 0;
}