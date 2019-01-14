#include <earth_rover_teensy_bridge/earth_rover_teensy_bridge.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, EarthRoverTeensyBridge::NODE_NAME);
    ros::NodeHandle nh("~");

    EarthRoverTeensyBridge broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
