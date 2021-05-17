#include "lidar_spherical_data_container/lidar_spherical_data_container.h"

#include <boost/make_shared.hpp>

namespace lidar_processing
{
    bool LidarSphericalDataContainer::init(uint32_t num_of_channels, uint32_t num_of_layers, uint32_t num_of_targets)
    {
        num_of_targets_ = num_of_targets;
        num_of_channels_ = num_of_channels;
        num_of_layers_ = num_of_layers;

        points_ = boost::make_shared<pcl::PointCloud<LidarPointXYZF>>(num_of_layers_*num_of_targets*num_of_channels_, 1U);

        return true;
    };

    bool LidarSphericalDataContainer::getCartesianPosition(uint32_t idx, float& x, float& y, float& z) const
    {
        if (idx >= num_of_points_)
        {
            x = y = z = 0.0;
            return false;
        }

        auto point = points_->at(idx);
        x = point.x;
        y = point.y;
        z = point.z;

        return true;
    };

    bool LidarSphericalDataContainer::getIdx(uint32_t channel_id, uint32_t layer_id, uint32_t target_id, uint32_t& idx) const
    {
        if (channel_id >= num_of_channels_ || layer_id >= num_of_layers_ || target_id >= num_of_layers_ )
        {
            idx = 0;
            return false;
        }

        uint32_t grid_idx = layer_id * num_of_targets_ * num_of_channels_ + channel_id * num_of_targets_ + target_id;
        idx = index_lookup_table_.at(grid_idx);
        return true;
    };

    bool LidarSphericalDataContainer::getFlag(uint32_t idx, uint8_t& flag) const
    {
        if (idx >= num_of_points_)
        {
            flag = 0;
            return false;
        }

        flag = points_->at(idx).flag;

        return true;
    };

    bool LidarSphericalDataContainer::getPointsIndsInChannel(uint32_t channel_id, std::vector<uint32_t>& ind_vec) const
    {
        if (channel_id >= num_of_channels_)
        {
            ind_vec.clear();
            return false;
        }

        ind_vec.clear();
        uint32_t first_idx = 0;
        if (channel_id > 0)
        {
            first_idx = channel_range_table_.at(channel_id - 1);
        }
        uint32_t last_idx = channel_range_table_.at(channel_id);
        ind_vec.reserve(last_idx - first_idx);

        for (; first_idx < last_idx; first_idx++)
        {
            ind_vec.push_back(first_idx);
        }
        return true;
    };
}

