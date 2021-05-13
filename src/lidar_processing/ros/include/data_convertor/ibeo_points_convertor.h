/* Helper class converting between ibeo pointcloud data and stixel container */

#pragma once
#include <inttypes.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "stixel_cylindrical_data_container/stixel_cylindrical_data_container.h"

struct PointXYZIRLDS
{
    PCL_ADD_POINT4D;                 
    float intensity;                 
    uint16_t ring;                  
    uint16_t label;                
    uint16_t device_id;              
    uint16_t segment_id;             
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRLDS,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label)
                                  (uint16_t, device_id, device_id)
                                  (uint16_t, segment_id, segment_id))

namespace lidar_processing
{
    class IbeoPointsConvertor
    {
        public:
            struct GridCell
            {
                float distance_xy;
                float theta;
                float cartesian_x;
                float cartesian_y;
                float min_height_z;
                float max_height_z;
                bool ground_measured;
                float ground_height_sum;
                uint32_t num_of_target_points;
                uint32_t num_of_ground_points;

                GridCell(): distance_xy(0.0),
                            theta(0.0),
                            cartesian_x(0.0),
                            cartesian_y(0.0),
                            min_height_z(0.0),
                            max_height_z(100.0),
                            ground_measured(false),
                            ground_height_sum(0.0),
                            num_of_target_points(0),
                            num_of_ground_points(0)
                {};
            };

            IbeoPointsConvertor(): max_range_(10.0),
                                   num_of_channels_(0),
                                   radial_resolution_(0.1),
                                   azimuth_resolution_(0.1)
            {};

            bool init(float max_range, float radial_resolution, float azimuth_resolution);

            bool convertPoints2Stixels(sensor_msgs::PointCloud2::ConstPtr ros_points, StixelCylindricalDataContainer& stixel_container_);

            static bool convertStixels2Points(const StixelCylindricalDataContainer& stixel_container, sensor_msgs::PointCloud2& ros_points);

        private:
            bool updateGridFromPoints(const pcl::PointCloud<PointXYZIRLDS>& points);

            bool extractDataFromGrid(StixelCylindricalDataContainer& stixel_container);

            bool isGround(uint32_t flag) const;

            bool isClutter(uint32_t flag) const;

            // cylindrical grid structure for pointcloud=>stixel
            std::vector<GridCell> grid_;
            // range of valid cell indices in each channel
            std::vector<std::pair<uint32_t, uint32_t>> grid_channel_range_; 

            // Grid meta data
            uint32_t max_range_;
            uint32_t num_of_cells_;
            uint32_t num_of_cells_radial_;

            // Stixel Container meta data
            uint32_t num_of_channels_;

            float radial_resolution_;
            float azimuth_resolution_;
    };
}