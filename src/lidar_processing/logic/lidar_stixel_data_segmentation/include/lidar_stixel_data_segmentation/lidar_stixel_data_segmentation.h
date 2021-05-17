/* A simple implementation of Streller-Dietmayer Segmentation Method */

#pragma once

#include "lidar_stixel_data_container/lidar_stixel_data_container.h"

#include <set>

namespace lidar_processing
{
    class LidarStixelDataSegmentation
    {
        public:
            LidarStixelDataSegmentation(): min_skippable_channel_(1),
                                           distance_scaling_factor_(0.0),
                                           measurement_noise_(0.0) {};

            bool init(const LidarStixelDataContainer& stixels);

            bool process(LidarStixelDataContainer& stixels);

        private:
            // private helper functions for segmentation process
            bool isInRange(float base_distance, float distance_to_cmp);

            bool fuseSegmentID(std::set<uint32_t> related_segments);

            // internal buffers for segmentation process
            std::vector<uint32_t> segmentation_linkage_lookup_;

            // meta properties for segmentation method
            static const int skippable_channel_radial_step_per_meter_ = 5;
            std::vector<uint32_t> skippable_channel_lookup_;
            float distance_scaling_factor_;
            float radial_resolution_;
    };
}