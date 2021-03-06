/* A template class describing a generic grid map data type. */

#pragma once

#include <math.h>
#include <cstring>
#include <vector>
#include <inttypes.h>
#include <iostream>
#include <limits>

namespace lidar_processing
{
    struct GridMetaData
    {
        uint32_t dim_x;
        uint32_t dim_y;
        double resolution;
        double origin_x;
        double origin_y;

        GridMetaData(): dim_x(0),
                        dim_y(0),
                        resolution(0.0),
                        origin_x(0.0),
                        origin_y(0.0) {}
    };
    
    template <typename CellType>
    class GridMap
    {
        public:

            GridMap(): initialized_(false) {};

            bool init(uint32_t dim_x, uint32_t dim_y, double resolution, double shift_distance)
            {
                grid_meta_data_.dim_x = dim_x;
                grid_meta_data_.dim_y = dim_y;

                if (resolution < std::numeric_limits<double>::epsilon())
                {
                    return false;
                }
                grid_meta_data_.resolution = resolution;

                double shift_distance_rounded = static_cast<int>(std::max(fabs(shift_distance) / resolution, 1.0) * resolution);
                shift_distance_sq_ = shift_distance_rounded * shift_distance_rounded;

                grids_.resize(grid_meta_data_.dim_x * grid_meta_data_.dim_y, CellType());

                return true;
            };

            inline const GridMetaData& getGridMetaData() {return grid_meta_data_;};

            bool updateEgoShift(double ego_x, double ego_y)
            {
                if (initialized_)
                {
                    double delta_ego_x = grid_meta_data_.origin_x + static_cast<double>(grid_meta_data_.dim_x) / 2.0 * grid_meta_data_.resolution - ego_x;
                    double delta_ego_y = grid_meta_data_.origin_y + static_cast<double>(grid_meta_data_.dim_y) / 2.0 * grid_meta_data_.resolution - ego_y;

                    if (delta_ego_x * delta_ego_x + delta_ego_y * delta_ego_y > shift_distance_sq_)
                    {
                        // shift grid
                        if (!shiftInX(delta_ego_x))
                        {
                            return false;
                        }

                        if (!shiftInY(delta_ego_y))
                        {
                            return false;
                        }
                    }
                }
                else
                {
                    grid_meta_data_.origin_x = ego_x - static_cast<double>(grid_meta_data_.dim_x) / 2.0 * grid_meta_data_.resolution;
                    grid_meta_data_.origin_y = ego_y - static_cast<double>(grid_meta_data_.dim_y) / 2.0 * grid_meta_data_.resolution;
                    initialized_ = true;
                }
                return true;
            };

            inline size_t size()
            {
                return grids_.size();
            }

            inline const CellType& at(size_t cell_idx) const
            {
                return grids_.at(cell_idx);
            }

            inline CellType& at(size_t cell_idx)
            {
                return grids_.at(cell_idx);
            }

        private:

            bool shiftInX(double delta_x)
            {
                int shift_grid_num = static_cast<int>(delta_x / grid_meta_data_.resolution);
                if (shift_grid_num == 0)
                {
                    return true;
                }

                std::vector<CellType> tmp_grids_(grids_.size(), CellType());

                int dst_idx_x = std::max(shift_grid_num, 0);
                int src_idx_x = std::max(-shift_grid_num, 0);
                int copy_len = std::min(grid_meta_data_.dim_x - shift_grid_num, grid_meta_data_.dim_x);

                if (dst_idx_x >= grid_meta_data_.dim_x || src_idx_x >= grid_meta_data_.dim_x || copy_len <= 0)
                {
                    grids_ = std::move(tmp_grids_);
                }
                else
                {
                    for (uint32_t idx_y = 0; idx_y < grid_meta_data_.dim_y; idx_y++)
                    {  
                        memcpy(grids_.data() + idx_y * grid_meta_data_.dim_x + dst_idx_x,
                               tmp_grids_.data() + idx_y * grid_meta_data_.dim_x + src_idx_x,
                               copy_len); 
                    }
                    grids_ = std::move(tmp_grids_);
                }
                
                grid_meta_data_.origin_x -= shift_grid_num * grid_meta_data_.resolution;
                return true;
            };

            bool shiftInY(double delta_y)
            {
                int shift_grid_num = static_cast<int>(delta_y / grid_meta_data_.resolution);
                if (shift_grid_num == 0)
                {
                    return true;
                }

                std::vector<CellType> tmp_grids_(grids_.size(), CellType());

                int dst_idx_y = std::max(shift_grid_num, 0);
                int src_idx_y = std::max(-shift_grid_num, 0);
                int copy_len = std::min(grid_meta_data_.dim_y - shift_grid_num, grid_meta_data_.dim_y) * grid_meta_data_.dim_x;

                if (dst_idx_y >= grid_meta_data_.dim_y || src_idx_y >= grid_meta_data_.dim_y || copy_len <= 0)
                {
                    grids_ = std::move(tmp_grids_);
                }
                else
                {
                    memcpy(grids_.data() + dst_idx_y * grid_meta_data_.dim_x,
                           tmp_grids_.data() + src_idx_y * grid_meta_data_.dim_x,
                           copy_len); 
                    grids_ = std::move(tmp_grids_);
                }
                
                grid_meta_data_.origin_y -= shift_grid_num * grid_meta_data_.resolution;
                return true;
            };

            GridMetaData grid_meta_data_;

            double shift_distance_sq_;
            
            std::vector<CellType> grids_;

            bool initialized_;
    };
}