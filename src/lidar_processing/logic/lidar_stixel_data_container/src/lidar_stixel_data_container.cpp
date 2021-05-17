#include "lidar_stixel_data_container/lidar_stixel_data_container.h"

#include <iostream>
#include <math.h>
#include <cstring>

namespace lidar_processing
{
    const float kMinResolution  = 0.01;

    bool LidarStixelDataContainer::init(float radial_resolution, float azimuth_resolution, uint32_t num_of_targets)
    {
        if (radial_resolution < kMinResolution || azimuth_resolution < kMinResolution)
        {
            return false;
        }

        radial_resolution_ = radial_resolution;
        azimuth_resolution_ = azimuth_resolution;

        num_of_channels_ = static_cast<uint32_t>(360.0 / radial_resolution_);
        num_of_targets_ = num_of_targets;
        num_of_layers_ = 1;
        stixel_capacity_ = num_of_channels_ * num_of_targets_ * num_of_layers_;
        num_of_stixels_ = 0;

        stixels_.resize(stixel_capacity_);
        memset(stixels_.data(), 0, sizeof(StixelTarget) * stixels_.size());

        return true;
    }

    bool LidarStixelDataContainer::getIdx(uint32_t channel_id, uint32_t layer_id, uint32_t target_id, uint32_t& idx) const
    {
        if (channel_id >= num_of_channels_ || layer_id >= num_of_layers_ || target_id >= num_of_targets_)
        {
            return false;
        }

        idx = layer_id * num_of_channels_ * num_of_targets_ + num_of_targets_ * channel_id + target_id;
        return true;
    }

    bool LidarStixelDataContainer::getIndsInChannel(uint32_t channel_id, std::vector<uint32_t>& inds) const
    {
        if (channel_id >= num_of_channels_)
        {
            inds.clear();
            return false;
        }

        inds.clear();
        inds.reserve(num_of_targets_ * num_of_layers_);
        for (uint32_t layer_id = 0; layer_id < num_of_layers_; layer_id++)
        {
            for (uint32_t target_id = 0; target_id < num_of_targets_; target_id++)
            {
                uint32_t idx = 0;
                if (getIdx(channel_id, layer_id, target_id, idx))
                {
                    inds.push_back(idx);
                }
            }
        }
        return true;
    }

    bool LidarStixelDataContainer::getFlag(uint32_t idx, uint8_t& flag) const
    {
        if (idx >= stixel_capacity_)
        {
            return false;
        }

        flag = stixels_.at(idx).flag;
        return true;
    }

    bool LidarStixelDataContainer::getDistance_xy(uint32_t idx, float& distance_xy) const
    {
        if (idx >= stixel_capacity_)
        {
            return false;
        }

        distance_xy = stixels_.at(idx).distance_xy;
        return true;
    }

    bool LidarStixelDataContainer::getTheta(uint32_t idx, float& theta) const
    {
        if (idx >= stixel_capacity_)
        {
            return false;
        }

        theta = stixels_.at(idx).theta;
        return true;
    }

    bool LidarStixelDataContainer::getCartesianPosition(uint32_t idx, float& x, float& y, float& z) const
    {
        if (idx >= stixel_capacity_)
        {
            return false;
        }

        x = stixels_.at(idx).cartesian_x;
        y = stixels_.at(idx).cartesian_y;
        z = stixels_.at(idx).cartesian_z;
        return true;
    }

    bool LidarStixelDataContainer::getDimension(uint32_t idx, float& height, float& width, float& length) const
    {
        if (idx >= stixel_capacity_)
        {
            return false;
        }

        height = stixels_.at(idx).dim_height;
        width = stixels_.at(idx).dim_width;
        length = radial_resolution_;
        return true;
    }

    bool LidarStixelDataContainer::getGroundHeight(uint32_t idx, float& ground_height) const
    {
        if (idx >= stixel_capacity_ || !stixels_.at(idx).ground_measured)
        {
            return false;
        }

        ground_height = stixels_.at(idx).groud_height;
        return true;
    }

    bool LidarStixelDataContainer::getSegmentID(uint32_t idx, uint32_t& segment_id) const
    {
        if (idx >= stixel_capacity_)
        {
            return false;
        }

        segment_id = stixels_.at(idx).segment_id;
        return true;
    }

    bool LidarStixelDataContainer::clear()
    {
        num_of_stixels_ = 0;
        memset(stixels_.data(), 0, sizeof(StixelTarget) * stixels_.size());
        return true;
    }

    bool LidarStixelDataContainer::setStixel(uint32_t idx, const StixelTarget& stixel)
    {
        if (idx >= stixel_capacity_)
        {
            return false;
        }

        uint8_t flag = 0;
        if (!getFlag(idx, flag))
        {
            return false;
        }

        if (stixel.flag & Flag_Target)
        {
            if (!(flag & Flag_Target))
            {
                // increase counter if it was not a target before
                num_of_stixels_++;
            }
            stixels_.at(idx) = stixel;
        }

        return true;
    }

}