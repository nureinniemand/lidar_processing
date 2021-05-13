#include "data_convertor/ibeo_points_convertor.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace lidar_processing
{
    class IbeoProcessor
    {
        public:
            IbeoProcessor() : nh_("~") {};

            bool init();

            bool run();

        private:
            void pointcloudCallback(sensor_msgs::PointCloud2::ConstPtr ros_points);

            ros::NodeHandle nh_;
            ros::Subscriber pointcloud_sub_;
            ros::Publisher stixel_pub_;

            StixelCylindricalDataContainer stixels_;
            IbeoPointsConvertor pointcloud_convertor_;
    };
}