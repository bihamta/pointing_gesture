#include <ros/ros.h>
#include <hands_3d/pointing_gesture.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pointing_gesture");
    ROS_INFO_STREAM("Reconstructor Initaited");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    PointingGesture pg(nh, pnh);

    ros::spin();
    return 0;
}
