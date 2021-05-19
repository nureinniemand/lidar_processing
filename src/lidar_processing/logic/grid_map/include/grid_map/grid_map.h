/* A template class describing a generic grid map data type. */

#pragma once

#include <math.h>

namespace lidar_processing
{
    template <typename CellType, typename PixelType, typename CellUpdator>
    class GridMap
    {
        public:
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

            GridMap(): updated_(false) {};

            bool init(uint32_t dim_x, uint32_t dim_y, double resolution, double shift_distance)
            {
                grid_meta_data_.dim_x = dim_x;
                grid_meta_data_.dim_y = dim_y;

                // TODO: a const value for epsilon
                if (resolution < 0.001)
                {
                    return false;
                }
                grid_meta_data_.resolution = resolution;

                double shift_distance_rounded = static_cast<int>(fabs(shift_distance) / resolution) * resolution;
                shift_distance_sq_ = shift_distance_rounded * shift_distance_rounded;

                grids_.resize(grid_meta_data_.dim_x * grid_meta_data_.dim_y, CellType());

                return true;
            };

            inline const GridMetaData& getGridMetaData() {return grid_meta_data_;};

            bool updateEgoShift(double ego_x, double ego_y)
            {
                if (updated_)
                {
                    double delta_ego_x = grid_meta_data_.origin_x + static_cast<double>(grid_meta_data_.dim_x) / 2.0 * resolution_ - ego_x;
                    double delta_ego_y = grid_meta_data_.origin_y + static_cast<double>(grid_meta_data_.dim_y) / 2.0 * resolution_ - ego_y;

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
                    grid_meta_data_.origin_x = ego_x - static_cast<double>(grid_meta_data_.dim_x) / 2.0 * resolution_;
                    grid_meta_data_.origin_y = ego_y - static_cast<double>(grid_meta_data_.dim_y) / 2.0 * resolution_;
                    updated_ = true;
                }
                return true;
            };

            bool updateFromImage(const PixelType* image, const GridMetaData& image_grid_meta)
            {
                // pre-check resolution, TODO: a const value for epsilon
                if (fabs(image_grid_meta.resolution - grid_meta_data_.resolution) > 0.001)
                {
                    std::cout << "The resolution between grid map and image does not match." << std::endl;
                    return false;
                }

                int dim_offset_x = static_cast<int>((image_grid_meta.origin_x - grid_meta_data_.origin_x) / grid_meta_data_.resolution);
                int dim_offset_y = static_cast<int>((image_grid_meta.origin_y - grid_meta_data_.origin_y) / grid_meta_data_.resolution);

                for (size_t cell_idx = 0; cell_idx < grids_.size(); cell_idx++)
                {
                    int image_idx_x = static_cast<int>(cell_idx % grid_meta_data_.dim_x) - dim_offset_x;
                    int image_idx_y = static_cast<int>(cell_idx % grid_meta_data_.dim_y) - dim_offset_y;
                    if (image_idx_x < 0 || image_idx_x > image_grid_meta.dim_x 
                     || image_idx_y < 0 || image_idx_y > image_grid_meta.dim_y)
                    {
                        continue;
                    }
                    cell_updator_(grids_.at(cell_idx), image[image_idx_y * image_grid_meta.dim_x + image_idx_x]);
                }
                return true;
            }

        private:

            bool shiftInX(double delta_x)
            {
                
                return true;
            };

            bool shiftInY(double delta_y)
            {
                return true;
            };

            GridMetaData grid_meta_data_;

            double shift_distance_sq_;
            
            std::vector<CellType> grids_;

            bool updated_;

            CellUpdator cell_updator_;
    };
}