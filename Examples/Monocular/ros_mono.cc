/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"System.h"

std::string image_topic_name = "/right_image";
std::string vocabulary_path = "Vocabulary/ORBvoc.txt";
std::string parameters_path = "sae.yaml";

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

void ReadParameters(ros::NodeHandle n) {
    std::string package_name = "orbslam2_ros";
    std::string pkg_path = ros::package::getPath(package_name)+"/";
    std::string parameter_name;
    std::string *str_ptr = nullptr;
    int *int_ptr = nullptr;
    double *double_ptr = nullptr;
    bool *bool_ptr = nullptr;

    parameter_name = "image_topic_name";
    str_ptr = &image_topic_name;
    n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    parameter_name = "vocabulary_path";
    str_ptr = &vocabulary_path;
    n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    parameter_name = "parameters_path";
    str_ptr = &parameters_path;
    n.getParam("/" + package_name + "/" + parameter_name, *str_ptr);

    if (vocabulary_path[0]!='/') vocabulary_path.insert(0,pkg_path);
    if (parameters_path[0]!='/') parameters_path.insert(0,pkg_path);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orbslam2_ros_mono");
    ros::NodeHandle n;
    ReadParameters(n);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocabulary_path, parameters_path, ORB_SLAM2::System::MONOCULAR, true);

    ImageGrabber igb(&SLAM);

    ros::Subscriber sub = n.subscribe(image_topic_name, 1, &ImageGrabber::GrabImage,&igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("/home/chao/KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}


