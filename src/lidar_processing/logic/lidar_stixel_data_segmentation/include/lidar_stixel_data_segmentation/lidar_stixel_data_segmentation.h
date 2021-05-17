/* A simple implementation of Streller-Dietmayer Segmentation Method */

#pragma once

#include "lidar_stixel_data_container/lidar_stixel_data_container.h"

#include <set>

namespace lidar_processing
{
    class LidarStixelDataSegmentation
    {
        public:
            LidarStixelDataSegmentation(): distance_scaling_factor_(0.0),
                                           radial_resolution_(0.1) {};

            bool init(const LidarStixelDataContainer& stixels, uint32_t min_skippable_channel, uint32_t max_compare_distance);

            bool process(LidarStixelDataContainer& stixels);

        private:
            // private helper functions for segmentation process
            bool isInRange(float base_distance, float distance_to_cmp);

            bool fuseSegmentId(std::set<uint32_t> related_segments);

            // internal buffers for segmentation process
            std::vector<uint32_t> segmentation_linkage_lookup_;

            // meta properties for segmentation method
            static const int skippable_channel_radial_step_per_meter_ = 5;
            std::vector<uint32_t> skippable_channel_lookup_;
            float distance_scaling_factor_;
            float radial_resolution_;
    };
}