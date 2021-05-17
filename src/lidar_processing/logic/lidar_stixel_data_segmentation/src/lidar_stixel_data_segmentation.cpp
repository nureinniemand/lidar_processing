#include "lidar_stixel_data_segmentation/lidar_stixel_data_segmentation.h"

#include <math.h>

namespace lidar_processing
{
    bool LidarStixelDataSegmentation::init(const LidarStixelDataContainer& stixels, uint32_t min_skippable_channel, uint32_t max_compare_distance)
    {
        const float azimuth_res = stixels.getAzimuthResolution();
        const float radial_res = stixels.getRadialResolution();
        const float max_range = stixels.getMaxRange();

        skippable_channel_lookup_.resize(static_cast<uint32_t>(max_range * skippable_channel_radial_step_per_meter_), 0);
        for (uint32_t i = 0; i < skippable_channel_lookup_.size(); i++)
        {
            skippable_channel_lookup_[i] = max(ceil(atan2(max_compare_distance, 
                                               static_cast<float>(i + 1) / skippable_channel_radial_step_per_meter_) / azimuth_res), 
                                               min_skippable_channel);
        }

        distance_scaling_factor_ = sqrt(2.0 - 2.0 * cos(azimuth_res / 180.0 * M_PI));
        radial_resolution_ = radial_res;

        segmentation_linkage_lookup_.resize(stixels.getNumOfStixels() + 1, 0);

        return true;
    }

    bool LidarStixelDataSegmentation::process(LidarStixelDataContainer& stixels)
    {
        // iterate through each channel
        for (uint32_t channel_id = 0; channel_id < stixels.getNumOfChannels(); channel_id++)
        {
            // iterate through each target in each channel
            for (uint32_t target_id = 0; target_id < stixels.getNumOfTargets(); target_id++)
            {
                uint32_t idx = 0;
                uint8_t flag = 0;
                float distance_xy = 0.0;
                if (!stixel.getIdx(channel_id, 0, target_id, idx)
                 || !stixel.getFlag(idx, flag) 
                 || (flag & LidarStixelDataContainer::Flag_Target == 0)
                 || !stixel.getDistance_xy(idx, distance_xy))
                {
                    // skip invalid target
                    continue;
                }

                std::set<uint32_t> related_segments;

                uint32_t skippable_channel = skippable_channel_lookup_[static_cast<uint32_t>(distance_xy * skippable_channel_radial_step_per_meter_)];
                int prev_channel_start_idx = static_cast<int>(channel_id) - 1;
                int prev_channel_end_idx = max(static_cast<int>(channel_id) - skippable_channel -1, -1);
                // iterate through all targets in the previous skippable channels
                for (; prev_channel_start_idx < prev_channel_end_idx; prev_channel_start_idx++)
                {
                    for (uint32_t prev_target_id = 0; prev_target_id < stixels.getNumOfTargets(); prev_target_id++)
                    {
                        uint32_t idx = 0;
                        uint8_t flag = 0;
                        float prev_distance_xy = 0.0;
                        uint32_t prev_segment_id = 0;
                        if (!stixel.getIdx(prev_channel_start_idx, 0, prev_target_id, idx)
                        || !stixel.getFlag(idx, flag) 
                        || (flag & LidarStixelDataContainer::Flag_Target == 0)
                        || !stixel.getDistance_xy(idx, prev_distance_xy)
                        || !stixel.getSegmentID(idx, ))
                        {
                            // skip invalid target
                            continue;
                        }

                        if (isInRange(distance_xy, prev_distance_xy))
                        {
                            related_segments.insert(prev_segment_id);
                        }
                    }
                }
            }
        }
        return true;
    }

    bool LidarStixelDataSegmentation::isInRange(float base_distance, float distance_to_cmp)
    {
        return fabs(base_distance - distance_to_cmp) < radial_resolution_ + distance_scaling_factor_ * max(base_distance, distance_to_cmp);
    }

    bool LidarStixelDataSegmentation::fuseSegmentID(std::set<uint32_t> related_segments)
    {
        return true;
    }
}