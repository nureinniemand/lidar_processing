/* A template class describing a generic grid map data type. */

#pragma once

namespace lidar_processing
{
    template <typename CellType, typename PixelType, typename CellUpdator>
    class GridMap
    {
        public:
            GridMap(): updated_(false) {};

            bool init(uint32_t dim_x, uint32_t dim_y, double resolution, double shift_distance)
            {
                dim_x_ = dim_x;
                dim_y_ = dim_y;
                resolution_ = resolution;
                shift_distance_sq_ = shift_distance * shift_distance;

                grids_.resize(dim_x_ * dim_y_, CellType());
            };

            bool updateEgoShift(double ego_x, double ego_y)
            {
                if (updated_)
                {
                    double delta_ego_x = origin_x_ + static_cast<double>(dim_x_) / 2.0 * resolution_ - ego_x;
                    double delta_ego_y = origin_y_ + static_cast<double>(dim_y_) / 2.0 * resolution_ - ego_y;

                    if ()
                }
                return true;
            };

            bool updateFromImage(const PixelType* image, )
            {
                return true;
            }

        private:
            double origin_x_;
            double origin_y_;

            uint32_t dim_x_;
            uint32_t dim_y_;

            double resolution_;

            double shift_distance_sq_;
            
            std::vector<CellType> grids_;

            bool updated_;
    };
}