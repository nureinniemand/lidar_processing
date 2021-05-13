#include "data_convertor/ibeo_points_convertor.h"

#include <math.h>

namespace lidar_processing
{
    const float kMinResolution  = 0.01;
    const float kMinGridRange = 1.0;

    bool IbeoPointsConvertor::init(float max_range, float radial_resolution, float azimuth_resolution)
    {
        if (radial_resolution < kMinResolution || azimuth_resolution < kMinResolution || max_range < kMinGridRange)
        {
            return false;
        }

        radial_resolution_ = radial_resolution;
        azimuth_resolution_ = azimuth_resolution;

        num_of_cells_radial_ = static_cast<uint32_t>(max_range / radial_resolution_);
        num_of_channels_ = static_cast<uint32_t>(360.0 / radial_resolution_);
        num_of_cells_ = num_of_channels_ * num_of_cells_radial_;

        grid_.clear();
        grid_.resize(num_of_cells_);

        return true;
    };

    bool IbeoPointsConvertor::convert_points_to_stixels(sensor_msgs::PointCloud2::ConstPtr ros_points, StixelCylindricalDataContainer& stixel_container)
    {
        // check the meta setting of convertor and container
        if (fabs(stixel_container.get_azimuth_resolution() - azimuth_resolution_) > kMinResolution ||
            fabs(stixel_container.get_radial_resolution() - radial_resolution_) > kMinResolution)
        {
            // need to re-init the stixel container based on convertor settings
            init(azimuth_resolution_, radial_resolution_, stixel_container.get_num_of_targets());
        }

        pcl::PointCloud<PointXYZIRLDS> pcl_points;
        pcl::fromROSMsg(*ros_points, pcl_points);

        if (!update_grid_from_points(pcl_points))
        {
            return false;
        }

        if (!extract_data_from_grid(stixel_container))
        {
            return false;
        }

        return true;
    };

    bool IbeoPointsConvertor::convert_stixels_to_points(const StixelCylindricalDataContainer& stixel_container, sensor_msgs::PointCloud2& ros_points)
    {
        pcl::PointCloud<PointXYZIRLDS> pcl_points(stixel_container.get_num_of_stixels(), 1U);

        for (uint32_t idx = 0; idx < stixel_container.get_num_of_stixels(); idx++)
        {
            PointXYZIRLDS& point = pcl_points.at(idx);
            stixel_container.get_cartesian_position(idx, point.x, point.y, point.z);

            uint8_t flag = 0;
            stixel_container.get_flag(idx, flag);
            point.label = static_cast<uint16_t>(flag);

            uint32_t segment_id = 0;
            stixel_container.get_segment_id(idx, segment_id);
            point.segment_id = static_cast<uint16_t>(segment_id);

            point.ring = 0;
            point.device_id = 0;
        }

        pcl::toROSMsg(pcl_points, ros_points);
        return true;
    }

    bool IbeoPointsConvertor::update_grid_from_points(pcl::PointCloud<PointXYZIRLDS>::ConstPtr points)
    {
        return true;
    }

    bool IbeoPointsConvertor::extract_data_from_grid(StixelCylindricalDataContainer& stixel_container)
    {
        return true;
    }
}