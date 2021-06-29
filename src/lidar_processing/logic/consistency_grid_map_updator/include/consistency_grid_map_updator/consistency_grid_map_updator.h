/* Updator of a grid map which CellType is ConsistencyGrid. */

#pragma once

#include "consistency_cell/consistency_cell.h"
#include "grid_map/grid_map.h"

#include "lidar_stixel_data_container/lidar_stixel_data_container.h"

namespace lidar_processing
{
    class ConsistencyGridMapUpdator
    {
        public:
            typedef uint8_t PixelType;

            struct UpdatorProperties
            {
                UpdatorProperties(): hit_overflow_thresh(1),
                                     miss_overflow_thresh(1)
                {

                };

                UpdatorProperties(const UpdatorProperties& update_properties): 
                    hit_overflow_thresh(update_properties.hit_overflow_thresh),
                    miss_overflow_thresh(update_properties.miss_overflow_thresh)
                {

                };

                uint8_t hit_overflow_thresh;
                uint8_t miss_overflow_thresh;
            };

            class ConsistencyCellUpdator
            {
                public:
                    ConsistencyCellUpdator() = default;
                    
                    ConsistencyCellUpdator(const UpdatorProperties& update_properties): 
                        update_properties_(update_properties)
                    {

                    };

                    bool operator() (ConsistencyCell& cell, uint8_t pixel_val) const;

                private:
                    UpdatorProperties update_properties_;
            };

            bool init();

            bool updateGridMapFromStixels(const LidarStixelDataContainer& stixels);

        private:
            bool updateFromImage();

            bool generateImageFromStixels(const LidarStixelDataContainer& stixels);

            GridMap<ConsistencyCell> grid_map_;

            GridMetaData image_grid_meta_;

            std::vector<PixelType> image_grid_map_;

            ConsistencyCellUpdator cell_updator_;
    };
}