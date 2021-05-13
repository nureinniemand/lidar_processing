/*************************************************************/
/* A container to store point cloud data from lidar sensors. */
/* It offers the interface to get each reflect point in a    */
/* [channel, layer, target] manner. Meanwhile, data is stored*/
/* in a chunked memory for efficiency.                       */
/*************************************************************/

#pragma once
#include <vector>
#include <inttypes.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct LidarPointXYZF
{
    PCL_ADD_POINT4D;
    uint8_t flag;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (LidarPointXYZF,   
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (uint8_t, flag, flat)
)

namespace lidar_processing
{
    class LidarSphericalDataContainer
    {
        public:
            typedef LidarPointXYZF tPointType;

            enum LidarPointFlag: uint8_t
            {
                Target = 1,
                Ground = 1 << 1,
                LessSensitive = 1 << 2
            };

            LidarSphericalDataContainer() : num_of_points_(0),
                                            num_of_channels_(0),
                                            num_of_layers_(0),
                                            num_of_targets_(0)
            {};

            bool init(uint32_t num_of_channels, uint32_t num_of_layers, uint32_t num_of_targets);

            // getter functions
            inline uint32_t get_num_of_points() const {return num_of_points_;};
            inline uint32_t get_num_of_channels() const {return num_of_channels_;};
            inline uint32_t get_num_of_layers() const {return num_of_layers_;};
            inline uint32_t get_num_of_targets() const {return num_of_targets_;};

            bool get_cartesian_position(uint32_t idx, float& x, float& y, float& z) const;
            bool get_idx(uint32_t channel_id, uint32_t layer_id, uint32_t target_id, uint32_t& idx) const; 
            bool get_flag(uint32_t idx, uint8_t& flag) const;

            bool get_points_inds_in_channel(uint32_t channel_id, std::vector<uint32_t>& ind_vec) const;

            // setter functions
            
        private:
            // memory chunks
            pcl::PointCloud<tPointType>::Ptr points_;

            // lookup [layer, channel, target] -> index
            std::vector<uint32_t> index_lookup_table_;

            // lookup channel_id -> last point index+1
            std::vector<uint32_t> channel_range_table_;

            // main attributes
            uint32_t num_of_points_;
            uint32_t num_of_channels_;
            uint32_t num_of_layers_;
            uint32_t num_of_targets_;
    };
}
