#include "pointcloud_segmentation/pointcloud_segmentation.h"

#include <chrono>
#include <iostream>
#include <string>

namespace lidar_processing
{
    bool PointCloudSegementation::init()
    {
        std::string ibeo_points_topic_in_name;
        nh_.param<std::string>("ibeo_points_topic_in", ibeo_points_topic_in_name, std::string("/raw/ibeo/point_cloud"));
        pointcloud_sub_ = nh_.subscribe(ibeo_points_topic_in_name, 1, &PointCloudSegementation::pointcloudCallback, this);

        std::string stixel_topic_out_name;
        nh_.param<std::string>("stixel_topic_out", stixel_topic_out_name, std::string("/raw/ibeo/stixels"));
        stixel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(stixel_topic_out_name, 1);

        float radial_resolution = 0.4, azimuth_resolution = 0.25;
        nh_.param<float>("stixel_radial_resolution", radial_resolution, 0.4);
        nh_.param<float>("stixel_azimuth_resolution", azimuth_resolution, 0.25);

        int num_of_targets = 0;
        nh_.param<int>("stixel_targets_number", num_of_targets, 5);

        float max_range = 50.0;
        nh_.param<float>("max_range", max_range, 80.0);

        int min_skippable_channel = 3;
        nh_.param<int>("min_skippable_channel", min_skippable_channel, 3);

        float max_compare_distance = 0.75;
        nh_.param<float>("max_compare_distance", max_compare_distance, 0.75);

        if (!stixels_.init(radial_resolution, azimuth_resolution, num_of_targets, max_range))
        {
            return false;
        }

        if (!pointcloud_convertor_.init(stixels_))
        {
            return false;
        }

        if (!pointcloud_segmentor_.init(stixels_, min_skippable_channel, max_compare_distance))
        {
            return false;
        }

        return true;
    }

    bool PointCloudSegementation::run()
    {
        ros::spin();
        return true;
    }

    void PointCloudSegementation::pointcloudCallback(sensor_msgs::PointCloud2::ConstPtr ros_points)
    {
        auto start = std::chrono::high_resolution_clock::now();
        std::cout << "Starting converting points to stixels..." << std::endl;
        if (!pointcloud_convertor_.convertPoints2Stixels(ros_points, stixels_))
        {
            return;
        }
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        auto elapsed_ms = std::chrono::duration_cast< std::chrono::milliseconds >(elapsed).count();
        std::cout << "point2stixel" << " => time cost: " << elapsed_ms << " milliseconds \n" << std::endl;

        start = std::chrono::high_resolution_clock::now();
        std::cout << "Starting stixel segmentation..." << std::endl;
        if (!pointcloud_segmentor_.process(stixels_))
        {
            return;
        }
        elapsed = std::chrono::high_resolution_clock::now() - start;
        elapsed_ms = std::chrono::duration_cast< std::chrono::milliseconds >(elapsed).count();
        std::cout << "segmentation" << " => time cost: " << elapsed_ms << " milliseconds \n" << std::endl;

        start = std::chrono::high_resolution_clock::now();
        std::cout << "Starting converting stixels to points..." << std::endl;
        sensor_msgs::PointCloud2 msg_to_pub;
        if (!pointcloud_convertor_.convertStixels2Points(stixels_, msg_to_pub))
        {
            return;
        }
        msg_to_pub.header = ros_points->header;
        stixel_pub_.publish(msg_to_pub);
        elapsed = std::chrono::high_resolution_clock::now() - start;
        elapsed_ms = std::chrono::duration_cast< std::chrono::milliseconds >(elapsed).count();
        std::cout << "stixel2point" << " => time cost: " << elapsed_ms << " milliseconds \n" << std::endl;

        
    }
}