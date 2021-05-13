#include "data_convertor/ibeo_points_convertor.h"

#include <math.h>
#include <cstring>

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

        max_range_ = max_range;
        radial_resolution_ = radial_resolution;
        azimuth_resolution_ = azimuth_resolution;

        num_of_cells_radial_ = static_cast<uint32_t>(max_range / radial_resolution_);
        num_of_channels_ = static_cast<uint32_t>(360.0 / azimuth_resolution_);
        num_of_cells_ = num_of_channels_ * num_of_cells_radial_;

        grid_.clear();
        grid_.resize(num_of_cells_);

        return true;
    };

    bool IbeoPointsConvertor::convertPoints2Stixels(sensor_msgs::PointCloud2::ConstPtr ros_points, StixelCylindricalDataContainer& stixel_container)
    {
        // check the meta setting of convertor and container
        if (fabs(stixel_container.getAzimuthResolution() - azimuth_resolution_) > kMinResolution ||
            fabs(stixel_container.getRadialResolution() - radial_resolution_) > kMinResolution)
        {
            // need to re-init the stixel container based on convertor settings
            init(azimuth_resolution_, radial_resolution_, stixel_container.getNumOfTargets());
        }

        pcl::PointCloud<PointXYZIRLDS> pcl_points;
        pcl::fromROSMsg(*ros_points, pcl_points);

        if (!updateGridFromPoints(pcl_points))
        {
            return false;
        }

        if (!extractDataFromGrid(stixel_container))
        {
            return false;
        }

        return true;
    };

    bool IbeoPointsConvertor::convertStixels2Points(const StixelCylindricalDataContainer& stixel_container, sensor_msgs::PointCloud2& ros_points)
    {
        pcl::PointCloud<PointXYZIRLDS> pcl_points(stixel_container.getNumOfStixels(), 1U);

        uint32_t num_of_points = 0;
        for (uint32_t idx = 0; 
            idx < stixel_container.getCapacity() && num_of_points < pcl_points.size(); 
            idx++)
        {
            uint8_t flag = 0;
            stixel_container.getFlag(idx, flag);
            if (flag & StixelCylindricalDataContainer::Flag_Target)
            {
                PointXYZIRLDS& point = pcl_points.at(num_of_points++);
                stixel_container.getCartesianPosition(idx, point.x, point.y, point.z);

                point.label = static_cast<uint16_t>(flag);

                uint32_t segment_id = 0;
                stixel_container.getSegmentID(idx, segment_id);
                point.segment_id = static_cast<uint16_t>(segment_id);

                point.ring = 0;
                point.device_id = 0;
            }
            
        }

        pcl::toROSMsg(pcl_points, ros_points);
        return true;
    }

    bool IbeoPointsConvertor::updateGridFromPoints(const pcl::PointCloud<PointXYZIRLDS>& points)
    {
        // reset grid data
        memset(grid_.data(), 0, sizeof(GridCell) * grid_.size());
        grid_channel_range_.resize(num_of_channels_, std::make_pair(num_of_cells_radial_-1, 0U));

        for (auto iter = points.begin(); iter != points.end(); iter++)
        {
            if (isClutter(iter->label))
            {
                continue;
            }

            // Calculate the cell index in the polar grid map
            float theta = atan2(iter->y, iter->x) + M_PI;
            uint32_t channel_id = static_cast<uint32_t>(theta / M_PI * 180.0 / azimuth_resolution_);
            float distance_xy = sqrt(iter->x * iter->x + iter->y * iter->y);
            if (distance_xy >= max_range_)
            {
                continue;
            }
            uint32_t radial_id = static_cast<uint32_t>(distance_xy / radial_resolution_);
            uint32_t idx_id = channel_id * num_of_cells_radial_ + radial_id;

            // update the grid information
            auto& cur_grid = grid_.at(idx_id);
            if (isGround(iter->label))
            {
                cur_grid.ground_measured = true;
                cur_grid.ground_height_sum += iter->z;
                cur_grid.num_of_ground_points++;
            }
            else
            {
                // std::cout << "Add one point into grid map" << std::endl;
                cur_grid.distance_xy = distance_xy;
                cur_grid.theta = theta;
                cur_grid.cartesian_x = iter->x;
                cur_grid.cartesian_y = iter->y;
                cur_grid.min_height_z = std::min(cur_grid.min_height_z, iter->z);
                cur_grid.max_height_z = std::max(cur_grid.max_height_z, iter->z);
                cur_grid.num_of_target_points++;

                // update channel range
                grid_channel_range_.at(channel_id).first = std::min(grid_channel_range_.at(channel_id).first, radial_id);
                grid_channel_range_.at(channel_id).second = std::max(grid_channel_range_.at(channel_id).second, radial_id);
            }
        }
        return true;
    }

    bool IbeoPointsConvertor::extractDataFromGrid(StixelCylindricalDataContainer& stixel_container)
    {
        // the meta setting of stixel and grid should be matched before this function.
        stixel_container.clear();

        for (uint32_t channel_id = 0; channel_id < num_of_channels_; channel_id++)
        {
            // get the radial range
            auto channel_range = grid_channel_range_.at(channel_id);
            if (channel_range.first > channel_range.second)
            {
                // empty channel, skip
                continue;
            }

            uint32_t num_targets = 0;
            uint32_t grid_channel_base_idx = channel_id * num_of_cells_radial_;
            for (uint32_t idx_in_channel = channel_range.first; 
                idx_in_channel <= channel_range.second;
                idx_in_channel++)
            {
                const auto& grid = grid_.at(grid_channel_base_idx + idx_in_channel);
                if (grid.num_of_target_points) // valid cell
                {
                    StixelCylindricalDataContainer::StixelTarget stixel;
                    stixel.flag = StixelCylindricalDataContainer::Flag_Target;
                    stixel.distance_xy = grid.distance_xy;
                    stixel.theta = grid.theta;
                    stixel.cartesian_x = grid.cartesian_x;
                    stixel.cartesian_y = grid.cartesian_y;
                    stixel.cartesian_z = grid.min_height_z;
                    stixel.dim_height = grid.max_height_z - grid.min_height_z;
                    if (grid.ground_measured && grid.num_of_ground_points)
                    {
                        stixel.ground_measured = true;
                        stixel.groud_height = grid.ground_height_sum / grid.num_of_ground_points;
                    }
                    num_targets++;

                    if (num_targets < stixel_container.getNumOfTargets())
                    {
                        uint32_t stixel_idx = 0;
                        if (!stixel_container.getIdx(channel_id, 0, num_targets-1, stixel_idx))
                        {
                            continue;
                        }
                        stixel_container.setStixel(stixel_idx, stixel);
                    }
                }
            }
        }

        return true;
    }

    bool IbeoPointsConvertor::isGround(uint32_t flag) const
    {
        // ESPF_Ground             = 0x0001 
        // ESPF_RoadMarking        = 0x0008 
        static const uint32_t ground_flag = (0x0001 | 0x0008);
        return (flag & ground_flag);
    }

    bool IbeoPointsConvertor::isClutter(uint32_t flag) const
    {
        // ESPF_Dirt               = 0x0002
        // ESPF_Rain               = 0x0004
        static const uint32_t clutter_flag = (0x0002 | 0x0004);
        return (flag & clutter_flag);
    }
}