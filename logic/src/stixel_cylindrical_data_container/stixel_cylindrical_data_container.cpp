#include "stixel_cylindrical_data_container/stixel_cylindrical_data_container.h"

#include <iostream>
#include <math.h>

namespace lidar_processing
{
    const float kMinResolution  = 0.01;

    bool StixelCylindricalDataContainer::init(float radial_resolution, float azimuth_resolution, uint32_t num_of_targets)
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
        num_of_stixels_ = num_of_channels_ * num_of_targets_ * num_of_layers_;

        stixels_.clear();
        stixels_.resize(num_of_stixels_);

        return true;
    }

    bool StixelCylindricalDataContainer::get_idx(uint32_t channel_id, uint32_t layer_id, uint32_t target_id, uint32_t& idx) const
    {
        if (channel_id >= num_of_channels_ || layer_id >= num_of_layers_ || target_id >= num_of_targets_)
        {
            return false;
        }

        idx = layer_id * num_of_channels_ * num_of_targets_ + num_of_targets_ * channel_id + target_id;
        return true;
    }

    bool StixelCylindricalDataContainer::get_inds_in_channel(uint32_t channel_id, std::vector<uint32_t>& inds) const
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
                if (get_idx(channel_id, layer_id, target_id, idx))
                {
                    inds.push_back(idx);
                }
            }
        }
        return true;
    }

    bool StixelCylindricalDataContainer::get_flag(uint32_t idx, uint8_t& flag) const
    {
        if (idx >= num_of_stixels_)
        {
            return false;
        }

        flag = stixels_.at(idx).flag;
        return true;
    }

    bool StixelCylindricalDataContainer::get_distance_xy(uint32_t idx, float& distance_xy) const
    {
        if (idx >= num_of_stixels_)
        {
            return false;
        }

        distance_xy = stixels_.at(idx).distance_xy;
        return true;
    }

    bool StixelCylindricalDataContainer::get_theta(uint32_t idx, float& theta) const
    {
        if (idx >= num_of_stixels_)
        {
            return false;
        }

        theta = stixels_.at(idx).theta;
        return true;
    }

    bool StixelCylindricalDataContainer::get_cartesian_position(uint32_t idx, float& x, float& y, float& z) const
    {
        if (idx >= num_of_stixels_)
        {
            return false;
        }

        x = stixels_.at(idx).cartesian_x;
        y = stixels_.at(idx).cartesian_y;
        z = stixels_.at(idx).cartesian_z;
        return true;
    }

    bool StixelCylindricalDataContainer::get_dimension(uint32_t idx, float& height, float& width, float& length) const
    {
        if (idx >= num_of_stixels_)
        {
            return false;
        }

        height = stixels_.at(idx).dim_height;
        width = stixels_.at(idx).dim_width;
        length = radial_resolution_;
        return true;
    }

    bool StixelCylindricalDataContainer::get_ground_height(uint32_t idx, float& ground_height) const
    {
        if (idx >= num_of_stixels_ || !stixels_.at(idx).ground_measured)
        {
            return false;
        }

        ground_height = stixels_.at(idx).groud_height;
        return true;
    }

    bool StixelCylindricalDataContainer::get_segment_id(uint32_t idx, uint32_t& segment_id) const
    {
        if (idx >= num_of_stixels_)
        {
            return false;
        }

        segment_id = stixels_.at(idx).segment_id;
        return true;
    }

}