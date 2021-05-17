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
            skippable_channel_lookup_[i] = std::max(static_cast<double>(ceil(atan2(max_compare_distance, 
                                                    static_cast<double>(i + 1) / skippable_channel_radial_step_per_meter_) / azimuth_res)), 
                                                    static_cast<double>(min_skippable_channel));
        }

        distance_scaling_factor_ = sqrt(2.0 - 2.0 * cos(azimuth_res / 180.0 * M_PI));
        radial_resolution_ = radial_res;

        segmentation_linkage_lookup_.resize(stixels.getNumOfStixels() + 1, 0);

        return true;
    }

    bool LidarStixelDataSegmentation::process(LidarStixelDataContainer& stixels)
    {
        uint32_t next_available_id = 1;
        for (uint32_t channel_id = 0; channel_id < stixels.getNumOfChannels(); channel_id++)
        {
            for (uint32_t target_id = 0; target_id < stixels.getNumOfTargets(); target_id++)
            {
                uint32_t idx = 0;
                stixels.getIdx(channel_id, 0, target_id, idx);

                uint8_t flag = 0;
                stixels.getFlag(idx, flag);

                if ((flag & LidarStixelDataContainer::Flag_Target) == 0)
                {
                    // reach the last target
                    break;
                }

                float distance_xy = 0.0;
                stixels.getDistance_xy(idx, distance_xy);

                std::set<uint32_t> related_segments;

                uint32_t skippable_channel = skippable_channel_lookup_[static_cast<uint32_t>(distance_xy * skippable_channel_radial_step_per_meter_)];
                int prev_channel_start_idx = static_cast<int>(channel_id) - 1;
                int prev_channel_end_idx = std::max(static_cast<int>(channel_id) - static_cast<int>(skippable_channel) -1, -1);
                // iterate through all targets in the previous skippable channels
                for (; prev_channel_start_idx < prev_channel_end_idx; prev_channel_start_idx++)
                {
                    for (uint32_t prev_target_id = 0; prev_target_id < stixels.getNumOfTargets(); prev_target_id++)
                    {
                        uint32_t prev_idx = 0;
                        stixels.getIdx(prev_channel_start_idx, 0, prev_target_id, prev_idx);
                        
                        uint8_t prev_flag = 0;
                        stixels.getFlag(prev_idx, prev_flag);
                        if ((prev_flag & LidarStixelDataContainer::Flag_Target) == 0)
                        {
                            break;
                        }

                        float prev_distance_xy = 0.0;
                        stixels.getDistance_xy(prev_idx, prev_distance_xy);

                        uint32_t prev_segment_id = 0;
                        stixels.getSegmentId(prev_idx, prev_segment_id);

                        if (isInRange(distance_xy, prev_distance_xy))
                        {
                            // insert the segmentation id in the prev channels in a set for id fusion
                            related_segments.insert(prev_segment_id);
                        }
                    }
                }

                if (related_segments.empty())
                {
                    // no related segments means its a new segment
                    stixels.setSegmentId(idx, next_available_id++);
                }
                else
                {
                    // has multiple linkage segments, need fusion
                    fuseSegmentId(related_segments);
                    stixels.setSegmentId(idx, *related_segments.cbegin());
                }
            } // for each target in the channel

            // compare each two targets in the channel
            for (uint32_t target_id = 0; target_id < stixels.getNumOfTargets(); target_id++)
            {
                std::set<uint32_t> related_segments;
                uint32_t idx = 0;
                stixels.getIdx(channel_id, 0, target_id, idx);

                uint8_t flag = 0;
                stixels.getFlag(idx, flag);
                if ((flag & LidarStixelDataContainer::Flag_Target) == 0)
                {
                    // reach the last target
                    break;
                }

                float distance_xy = 0.0;
                stixels.getDistance_xy(idx, distance_xy);

                for (uint32_t next_target_id = target_id + 1; next_target_id < stixels.getNumOfTargets(); next_target_id++)
                {
                    uint32_t next_idx = 0;
                    stixels.getIdx(channel_id, 0, next_target_id, next_idx);
                    
                    uint8_t next_flag = 0;
                    stixels.getFlag(next_idx, next_flag);
                    if ((next_flag & LidarStixelDataContainer::Flag_Target) == 0)
                    {
                        break;
                    }

                    float next_distance_xy = 0.0;
                    stixels.getDistance_xy(next_idx, next_distance_xy);

                    uint32_t next_segment_id = 0;
                    stixels.getSegmentId(next_idx, next_segment_id);

                    if (isInRange(distance_xy, next_distance_xy))
                    {
                        // insert the segmentation id for id fusion
                        related_segments.insert(next_segment_id);
                    }
                }

                if (!related_segments.empty())
                {
                    // has multiple linkage segments, need fusion
                    fuseSegmentId(related_segments);
                    stixels.setSegmentId(idx, *related_segments.cbegin());
                }
            }

        } // for each channel

        // clean linkage lookup for final segment id

        return true;
    }

    bool LidarStixelDataSegmentation::isInRange(float base_distance, float distance_to_cmp)
    {
        return fabs(base_distance - distance_to_cmp) < radial_resolution_ + distance_scaling_factor_ * std::max(base_distance, distance_to_cmp);
    }

    bool LidarStixelDataSegmentation::fuseSegmentId(std::set<uint32_t> related_segments)
    {
        const uint32_t& base_segment_id = *related_segments.cbegin();

        const bool is_base_related = segmentation_linkage_lookup_[base_segment_id] == 0;

        for (auto iter = ++related_segments.begin(); iter != related_segments.end(); iter++)
        {
            const bool is_segment_related = segmentation_linkage_lookup_[*iter];
            if (!is_base_related && !is_segment_related)
            {
                segmentation_linkage_lookup_[*iter] = base_segment_id;
            }
            else if (!is_segment_related)
            {
                uint32_t segment_id = base_segment_id;
                while (segmentation_linkage_lookup_[segment_id] != 0)
                {
                    segment_id = segmentation_linkage_lookup_[segment_id];
                }

                if (segment_id != *iter)
                {
                    segmentation_linkage_lookup_[*iter] = segment_id;
                }
            }
            else if (!is_base_related)
            {
                uint32_t segment_id = *iter;
                while (segmentation_linkage_lookup_[segment_id] != 0)
                {
                    segment_id = segmentation_linkage_lookup_[segment_id];
                }

                if (segment_id != base_segment_id)
                {
                    segmentation_linkage_lookup_[base_segment_id] = segment_id;
                }
            }
            else
            {
                std::set<uint32_t> all_related_segments;

                uint32_t segment_id = base_segment_id;
                all_related_segments.insert(segment_id);
                while (segmentation_linkage_lookup_[segment_id] != 0)
                {
                    segment_id = segmentation_linkage_lookup_[segment_id];
                    all_related_segments.insert(segment_id);
                }

                segment_id = *iter;
                all_related_segments.insert(segment_id);
                while (segmentation_linkage_lookup_[segment_id] != 0)
                {
                    segment_id = segmentation_linkage_lookup_[segment_id];
                    all_related_segments.insert(segment_id);
                }

                segmentation_linkage_lookup_[base_segment_id] = 0;
                for (const auto& seg_id: all_related_segments)
                {
                    if (seg_id != base_segment_id)
                    {
                        segmentation_linkage_lookup_[seg_id] = base_segment_id;
                    }
                }                
            }
        }
        return true;
    }
}