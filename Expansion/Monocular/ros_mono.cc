#include <ros/ros.h>

#include <VisualSLAM.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "orbslam2_ros_mono");
    ros::NodeHandle n("~");

    VisualSLAM vslam(n);
    int count = 0;
    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        count++;
    }

    vslam.Shutdown();
    ros::shutdown();
    return 0;
}




