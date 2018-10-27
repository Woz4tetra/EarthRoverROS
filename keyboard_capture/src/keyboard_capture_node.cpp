#include <keyboard_capture/keyboard_capture.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_capture");

    ros::NodeHandle nh("~");
    KeyboardCapture keyboard_capture(&nh);

    signal(SIGINT, quit);

    keyboard_capture.keyLoop();

    return(0);
}
