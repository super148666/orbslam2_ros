//
// Created by cz on 27/03/19.
//

#include <Expansion/include/VisualSLAM.h>


void VisualSLAM::GrabImage(const sensor_msgs::ImageConstPtr &msg) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    auto time_stamp = ros::Time::now();
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image, time_stamp.toSec());
    publisher->publish(Tcw, time_stamp);
}

void VisualSLAM::ReadParameters() {
    std::string package_name = "orbslam2_ros";
    std::string pkg_path = ros::package::getPath(package_name) + "/";

    n.param("/" + package_name + "/" + "image_topic_name", image_topic_name, std::string("/right_image"));

    n.param("/" + package_name + "/" + "vocabulary_path", vocabulary_path, std::string("Vocabulary/ORBvoc.txt"));

    n.param("/" + package_name + "/" + "parameters_path", parameters_path, std::string("sae.yaml"));

    save_map_file.clear();
    load_map_file.clear();
    n.param("/" + package_name + "/" + "save_map_file", save_map_file, std::string());
    n.param("/" + package_name + "/" + "load_map_file", load_map_file, std::string("/home/cz/map/01.map"));
    ROS_INFO_STREAM("load:" << load_map_file);
    ROS_INFO_STREAM("save:" << save_map_file);

    if (vocabulary_path[0] != '/') vocabulary_path.insert(0, pkg_path);
    if (parameters_path[0] != '/') parameters_path.insert(0, pkg_path);

    sub = n.subscribe(image_topic_name, 1, &VisualSLAM::GrabImage, this);
}

VisualSLAM::VisualSLAM(ros::NodeHandle n) : n(n) {
    ReadParameters();

    ROS_INFO_STREAM("load:" << load_map_file);
    if (!load_map_file.empty()) {
        std::ifstream ifs(load_map_file.c_str());
        boost::archive::binary_iarchive ia(ifs);
        ia & mpSLAM;
        auto voc = new ORBVocabulary;
        voc->loadFromTextFile(load_map_file+".voc");
        mpSLAM->SetORBVocabulary(voc);

    } else {
        mpSLAM = new ORB_SLAM2::System(vocabulary_path, parameters_path, ORB_SLAM2::System::MONOCULAR, false);
    }

    publisher = new Publisher(mpSLAM->mpMap, n);

}

void VisualSLAM::Shutdown() {

    ROS_INFO_STREAM("save:" << save_map_file);
    if (!save_map_file.empty()) {
        mpSLAM->SaveORBVocabulary(save_map_file+".voc");
        std::ofstream ofs(save_map_file.c_str(), ios_base::out | ios_base::trunc | ios::binary);
        boost::archive::binary_oarchive oa(ofs);
        oa & mpSLAM;
    }
    mpSLAM->Shutdown();
}
