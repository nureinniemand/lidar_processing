/* A simple implementation of Streller-Diemty Segmentation Method */

#pragma once

#include "lidar_stixel_data_container/lidar_stixel_data_container.h"

namespace lidar_processing
{
    class LidarStixelDataSegmentation
    {
        public:
            LidarStixelDataSegmentation() {};

            bool init();

            bool process();
    };
}