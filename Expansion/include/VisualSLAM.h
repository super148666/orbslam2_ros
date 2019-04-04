//
// Created by cz on 27/03/19.
//

#ifndef ORBSLAM2_ROS_VISUALSLAM_H
#define ORBSLAM2_ROS_VISUALSLAM_H


#include <ros/ros.h>
#include <ros/package.h>
#include <tf2/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <opencv2/core/core.hpp>

#include <Publisher.h>
#include "System.h"
#include "Converter.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class VisualSLAM {
public:
    explicit VisualSLAM(ros::NodeHandle n);

    void GrabImage(const sensor_msgs::ImageConstPtr &msg);

    ORB_SLAM2::System *mpSLAM;

    void Shutdown();

public:
    Publisher *publisher;

    void ReadParameters();

    std::string save_map_file;
    std::string load_map_file;
    std::string image_topic_name;
    std::string vocabulary_path;
    std::string parameters_path;
    ros::NodeHandle n;
    ros::Subscriber sub;
};


#endif //ORBSLAM2_ROS_VISUALSLAM_H
