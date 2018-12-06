#include <earth_rover_lidar_guard/earth_rover_lidar_guard.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "earth_rover_lidar_guard");

    ros::NodeHandle nh("~");

    EarthRoverLidarGuard earth_rover_lidar_guard(&nh);
    // ros::spin();

    return earth_rover_lidar_guard.run();
}
