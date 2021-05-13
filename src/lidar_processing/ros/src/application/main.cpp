#include "application/ibeo_processor.h"

int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "ibeo_processing_node");
    lidar_processing::IbeoProcessor main_processor;
    main_processor.init();
    main_processor.run();
    return 0;
}