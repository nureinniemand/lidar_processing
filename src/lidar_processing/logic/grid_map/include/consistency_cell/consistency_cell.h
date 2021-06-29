/* A specialized grid cell type for dynamic and static environment modeling */

#pragma once

#include <inttypes.h>

namespace lidar_processing
{
    class ConsistencyCell
    {
        public:
            enum ConsistencyCellState: uint8_t
            {
                State_Unknown = 0,
                State_StaticObject,
                State_Free,
                State_Inconsistency,
                State_DynObject,
                State_NUMS_
            };

            enum ConsistencyPixelFlag: uint8_t
            {
                Pixel_NonHit = 0,
                Pixel_DirectHit = 1,
                Pixel_ObjectHit = 1 << 1
            };

            ConsistencyCell(): hit_counter(0),
                               miss_counter(0),
                               cell_state(State_Unknown) 
            {

            };

            uint8_t hit_counter;
            uint8_t miss_counter;
            ConsistencyCellState cell_state;
    };
}