#pragma once
#include "data_convertor/ibeo_points_convertor.h"
#include "pointcloud_segmentation/pointcloud_segmentation.h"
#include "lidar_stixel_data_segmentation/lidar_stixel_data_segmentation.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace lidar_processing
{
    class PointCloudSegementation
    {
        public:
            PointCloudSegementation() : nh_("~") {};

            bool init();

            bool run();

        private:
            void pointcloudCallback(sensor_msgs::PointCloud2::ConstPtr ros_points);

            ros::NodeHandle nh_;
            ros::Subscriber pointcloud_sub_;
            ros::Publisher stixel_pub_;

            LidarStixelDataContainer stixels_;
            IbeoPointsConvertor pointcloud_convertor_;
            LidarStixelDataSegmentation pointcloud_segmentor_;
    };
}