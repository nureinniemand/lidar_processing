#include "consistency_grid_map_updator/consistency_grid_map_updator.h"

#include <limits>

namespace lidar_processing
{
    bool ConsistencyGridMapUpdator::ConsistencyCellUpdator::operator() (ConsistencyCell& cell, uint8_t pixel_val) const
    {
        // update the cell counter and state
        bool hit_overflow = false, miss_overflow = false, object_hit = false;
        if (pixel_val)
        {
            if (pixel_val & ConsistencyCell::Pixel_ObjectHit)
            {
                object_hit = true;
            }
            cell.hit_counter++;
            if (cell.hit_counter >= update_properties_.hit_overflow_thresh)
            {
                hit_overflow = true;
                cell.hit_counter = update_properties_.hit_overflow_thresh;
            }
        }
        else
        {
            cell.miss_counter++;
            if (cell.miss_counter >= update_properties_.miss_overflow_thresh)
            {
                miss_overflow = true;
                cell.miss_counter = update_properties_.miss_overflow_thresh;
            }
        }

        // update state machine
        switch (cell.cell_state)
        {
            case ConsistencyCell::State_Unknown:
            {
                if (hit_overflow)
                {
                    cell.cell_state = ConsistencyCell::State_StaticObject;
                } 
                else if (miss_overflow && cell.hit_counter > 0)
                {
                    cell.cell_state = ConsistencyCell::State_Inconsistency;
                }
                else if (miss_overflow)
                {
                    cell.cell_state = ConsistencyCell::State_Free;
                }
                break;
            }
            case ConsistencyCell::State_StaticObject:
            {
                if (miss_overflow)
                {
                    cell.cell_state = ConsistencyCell::State_Inconsistency;
                } 
                break;
            }
            case ConsistencyCell::State_Free:
            {
                if (cell.hit_counter > 0)
                {
                    cell.cell_state = ConsistencyCell::State_Inconsistency;
                } 
                break;
            }
            case ConsistencyCell::State_Inconsistency:
            {
                if (hit_overflow)
                {
                    cell.cell_state = ConsistencyCell::State_StaticObject;
                } 
                else if (object_hit)
                {
                    cell.cell_state = ConsistencyCell::State_DynObject;
                }
                break;
            }
            case ConsistencyCell::State_DynObject:
            {
                break;
            }
            default:
            {
                // do nothing
            }
        }
        return true;
    };

    bool ConsistencyGridMapUpdator::updateFromImage()
    {
        const GridMetaData grid_meta = grid_map_.getGridMetaData();
        if (fabs(image_grid_meta_.resolution - grid_meta.resolution) > std::numeric_limits<double>::epsilon())
        {
            std::cout << "The resolution between grid map and image does not match." << std::endl;
            return false;
        }

        int dim_offset_x = static_cast<int>((image_grid_meta_.origin_x - grid_meta.origin_x) / grid_meta.resolution);
        int dim_offset_y = static_cast<int>((image_grid_meta_.origin_y - grid_meta.origin_y) / grid_meta.resolution);

        for (size_t cell_idx = 0; cell_idx < grid_map_.size(); cell_idx++)
        {
            int image_idx_x = static_cast<int>(cell_idx % grid_meta.dim_x) - dim_offset_x;
            int image_idx_y = static_cast<int>(cell_idx % grid_meta.dim_y) - dim_offset_y;
            if (image_idx_x < 0 || image_idx_x > static_cast<int>(image_grid_meta_.dim_x) 
                || image_idx_y < 0 || image_idx_y > static_cast<int>(image_grid_meta_.dim_y))
            {
                continue;
            }
            cell_updator_(grid_map_.at(cell_idx), image_grid_map_[image_idx_y * image_grid_meta_.dim_x + image_idx_x]);
        }
        return true;
    }
}