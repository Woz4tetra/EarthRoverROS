#include <earth_rover_microcontroller_bridge/earth_rover_microcontroller_bridge.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, EarthRoverMicroControllerBridge::NODE_NAME);
    ros::NodeHandle nh("~");

    EarthRoverMicroControllerBridge broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
