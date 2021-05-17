/*******************************************************************/
/* A container to store general stixel data generated from lidar   */
/* point cloud. Data is stored in the 2.5D cylindical coodinate    */
/* manner.                                                         */
/*******************************************************************/

#pragma once
#include <inttypes.h>
#include <vector>

namespace lidar_processing
{
    class LidarStixelDataContainer
    {
        public:
            struct StixelTarget
            {
                uint8_t flag;
                float distance_xy;
                float theta;
                float cartesian_x;
                float cartesian_y;
                float cartesian_z; // lowest height of stixel
                float dim_height;
                float dim_width;
                float groud_height;
                bool ground_measured;
                uint32_t segment_id;

                StixelTarget(): flag(0),
                                distance_xy(0.0),
                                theta(0.0),
                                cartesian_x(0.0),
                                cartesian_y(0.0),
                                cartesian_z(0.0),
                                dim_height(0.0),
                                dim_width(0.0),
                                groud_height(0.0),
                                ground_measured(false),
                                segment_id(0)
                {};
            };

            enum StixelFlag: uint8_t
            {
                Flag_Target = 1,
                Flag_LessSensitive = 1 << 2
            };

            LidarStixelDataContainer(): num_of_channels_(0),
                                        num_of_layers_(0),
                                        num_of_targets_(0),
                                        num_of_stixels_(0),
                                        radial_resolution_(0.1),
                                        azimuth_resolution_(0.1),
                                        max_range_(0.1)
            {};

            bool init(float radial_resolution, float azimuth_resolution, uint32_t num_of_targets, float max_range);

            // getter functions
            inline uint32_t getNumOfStixels() const {return num_of_stixels_;};
            inline uint32_t getCapacity() const {return stixel_capacity_;};
            inline uint32_t getNumOfChannels() const {return num_of_channels_;};
            inline uint32_t getNumOfLayers() const {return num_of_layers_;};
            inline uint32_t getNumOfTargets() const {return num_of_targets_;};
            inline float getRadialResolution() const {return radial_resolution_;};
            inline float getAzimuthResolution() const {return azimuth_resolution_;};
            inline float getMaxRange() const {return max_range_;};

            bool getIdx(uint32_t channel_id, uint32_t layer_id, uint32_t target_id, uint32_t& idx) const; 

            bool getIndsInChannel(uint32_t channel_id, std::vector<uint32_t>& inds) const;

            bool getFlag(uint32_t idx, uint8_t& flag) const;
            bool getDistance_xy(uint32_t idx, float& distance_xy) const;
            bool getTheta(uint32_t idx, float& theta) const;
            bool getCartesianPosition(uint32_t idx, float& x, float& y, float& z) const;
            bool getDimension(uint32_t idx, float& height, float& width, float& length) const;
            bool getGroundHeight(uint32_t idx, float& ground_height) const;
            bool getSegmentId(uint32_t idx, uint32_t& segment_id) const;

            // setter functions
            bool clear();

            bool setStixel(uint32_t idx, const StixelTarget& stixel);
            bool setSegmentId(uint32_t idx, uint32_t segment_id);

        private:
            std::vector<StixelTarget> stixels_;

            uint32_t num_of_channels_;
            uint32_t num_of_layers_;
            uint32_t num_of_targets_;
            uint32_t stixel_capacity_;

            uint32_t num_of_stixels_;

            float radial_resolution_;
            float azimuth_resolution_;

            float max_range_;
    };
}