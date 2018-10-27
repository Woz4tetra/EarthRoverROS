#include <keyboard_capture/keyboard_capture.h>

KeyboardCapture::KeyboardCapture(ros::NodeHandle* nodehandle):
linear_(0),
angular_(0),
l_scale_(1.0),
a_scale_(1.0),
nh_(*nodehandle)
{
    nh_.param<double>("scale_angular", a_scale_, a_scale_);
    nh_.param<double>("scale_linear", l_scale_, l_scale_);

    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void KeyboardCapture::keyLoop()
{
    char c;
    bool dirty=false;


    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");


    for(;;)
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        linear_=angular_=0;
        ROS_DEBUG("value: 0x%02X\n", c);

        switch(c)
        {
            case KEYCODE_L:
                ROS_DEBUG("LEFT");
                angular_ = 1.0;
                dirty = true;
                break;
            case KEYCODE_R:
                ROS_DEBUG("RIGHT");
                angular_ = -1.0;
                dirty = true;
                break;
            case KEYCODE_U:
                ROS_DEBUG("UP");
                linear_ = 1.0;
                dirty = true;
                break;
            case KEYCODE_D:
                ROS_DEBUG("DOWN");
                linear_ = -1.0;
                dirty = true;
                break;
            default:
                ROS_DEBUG("STOPPING");
                // ROS_INFO("%d", c);
                linear_ = 0.0;
                angular_ = 0.0;
                dirty = true;
                break;
        }


        geometry_msgs::Twist twist;
        twist.angular.z = a_scale_*angular_;
        twist.linear.x = l_scale_*linear_;
        if(dirty ==true)
        {
            twist_pub_.publish(twist);
            dirty=false;
        }
    }


    return;
}
