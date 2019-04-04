// file: cvmat_serialization.h


#ifndef _CV_MAT_SERIALIZER
#define _CV_MAT_SERIALIZER

#include <opencv2/opencv.hpp>
#include "MapObjectSerialization.h"
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <ORBVocabulary.h>

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)
namespace boost {
    namespace serialization {

        /** Serialization support for cv::Mat */
        template<class Archive>
        void save(Archive &ar, const cv::Mat &m, const unsigned int version) {
            size_t elem_size = m.elemSize();
            size_t elem_type = m.type();

            ar & m.cols;
            ar & m.rows;
            ar & elem_size;
            ar & elem_type;

            const size_t data_size = m.cols * m.rows * elem_size;
            ar & boost::serialization::make_array(m.ptr(), data_size);
        }

        /** Serialization support for cv::Mat */
        template<class Archive>
        void load(Archive &ar, cv::Mat &m, const unsigned int version) {
            int cols, rows;
            size_t elem_size, elem_type;

            ar & cols;
            ar & rows;
            ar & elem_size;
            ar & elem_type;

            m.create(rows, cols, elem_type);

            size_t data_size = m.cols * m.rows * elem_size;
            ar & boost::serialization::make_array(m.ptr(), data_size);
        }

    }
}


BOOST_SERIALIZATION_SPLIT_FREE(std::vector<cv::Mat>)
namespace boost {
    namespace serialization {

        /** Serialization support for cv::Mat */
        template<class Archive>
        void save(Archive &ar, const std::vector<cv::Mat> &vm, const unsigned int version) {
            size_t sz = vm.size();
            ar & sz;
            for(auto&m:vm){
                ar & m;
            }
        }

        /** Serialization support for cv::Mat */
        template<class Archive>
        void load(Archive &ar, std::vector<cv::Mat> &vm, const unsigned int version) {
            size_t sz;
            ar & sz;
            for (size_t i=0;i<sz;i++){
                cv::Mat m;
                ar & m;
                vm.push_back(m);
            }
        }

    }
}


BOOST_SERIALIZATION_SPLIT_FREE(cv::KeyPoint)
namespace boost {
    namespace serialization {

        template<class Archive>
        void save(Archive &ar, const cv::KeyPoint &kp, const unsigned int version) {
            float kx = kp.pt.x,
                    ky = kp.pt.y;

            ar & kx;
            ar & ky;
            ar & kp.size;
            ar & kp.angle;
            ar & kp.response;
            ar & kp.octave;
            ar & kp.class_id;
        }

        template<class Archive>
        void load(Archive &ar, cv::KeyPoint &kp, const unsigned int version) {
            float kx, ky;

            ar & kx;
            ar & ky;
            ar & kp.size;
            ar & kp.angle;
            ar & kp.response;
            ar & kp.octave;
            ar & kp.class_id;
            kp.pt.x = kx;
            kp.pt.y = ky;
        }
    }
}

BOOST_SERIALIZATION_SPLIT_FREE(std::vector<cv::Point2f>)
namespace boost {
    namespace serialization {

        template<class Archive>
        void save(Archive &ar, const std::vector<cv::Point2f> &vp, const unsigned int version) {
            auto sz = vp.size();
            ar & sz;
            for (auto& p:vp) {
                float x = p.x, y = p.y;
                ar & x & y;
            }
        }

        template<class Archive>
        void load(Archive &ar, std::vector<cv::Point2f> &vp, const unsigned int version) {
            size_t sz;
            ar & sz;
            vp.clear();
            for (size_t i = 0; i < sz; i ++) {
                float x, y;
                ar & x & y;
                vp.emplace_back(x, y);
            }
        }
    }
}

BOOST_SERIALIZATION_SPLIT_FREE(std::vector<cv::Point3f>)
namespace boost {
    namespace serialization {

        template<class Archive>
        void save(Archive &ar, const std::vector<cv::Point3f> &vp, const unsigned int version) {
            auto sz = vp.size();
            ar & sz;
            for (auto& p:vp) {
                float x = p.x, y = p.y, z = p.z;
                ar & x & y & z;
            }
        }

        template<class Archive>
        void load(Archive &ar, std::vector<cv::Point3f> &vp, const unsigned int version) {
            size_t sz;
            ar & sz;
            vp.clear();
            for (size_t i = 0; i < sz; i ++) {
                float x, y, z;
                ar & x;
                ar & y;
                ar & z;
                vp.emplace_back(x, y, z);
            }
        }
    }
}


BOOST_SERIALIZATION_SPLIT_FREE(std::vector<cv::Point>)
namespace boost {
    namespace serialization {

        template<class Archive>
        void save(Archive &ar, const std::vector<cv::Point> &vp, const unsigned int version) {
            size_t sz = vp.size();
            ar & sz;
            for (auto& p:vp) {
                ar & p.x & p.y;
            }
        }

        template<class Archive>
        void load(Archive &ar, std::vector<cv::Point> &vp, const unsigned int version) {
            size_t sz;
            ar & sz;
            vp = std::vector<cv::Point>();
            for (size_t i = 0; i < sz; i ++) {
                int x, y;
                ar & x & y;
                vp.emplace_back(x, y);
            }
        }
    }
}


namespace ORB_SLAM2 {
    class KeyFrame;
}
typedef std::pair<std::set<ORB_SLAM2::KeyFrame *>,int> ConsistentGroup;
BOOST_SERIALIZATION_SPLIT_FREE(ConsistentGroup)
template<typename MapObject>
std::set<uint64_t> createIdList(const std::set<MapObject *> &mapObjectList);
template<typename MapObject>
std::set<MapObject *> createObjectList(const std::set<uint64_t> &mapIdList);
namespace boost {
    namespace serialization {

        template<class Archive>
        void save(Archive &ar, const ConsistentGroup &cg, const unsigned int version) {
            auto cgf_ids = createIdList(cg.first);
            ar & cgf_ids & cg.second;
        }

        template<class Archive>
        void load(Archive &ar, ConsistentGroup &cg, const unsigned int version) {
            std::set<uint64_t> cgf_ids;
            ar & cgf_ids & cg.second;
            cg.first = createObjectList<ORB_SLAM2::KeyFrame>(cgf_ids);
        }
    }
}

BOOST_SERIALIZATION_SPLIT_FREE(std::vector<ConsistentGroup>)
namespace boost {
    namespace serialization {

        template<class Archive>
        void save(Archive &ar, const std::vector<ConsistentGroup> &vcg, const unsigned int version) {
            auto sz = vcg.size();
            ar & sz;
            for (auto&cg:vcg){
                ar&cg;
            }
        }

        template<class Archive>
        void load(Archive &ar, std::vector<ConsistentGroup> &vcg, const unsigned int version) {
            size_t sz;
            ar & sz;
            for (size_t i=0;i<sz;i++){
                ConsistentGroup cg;
                ar & cg;
                vcg.push_back(cg);
            }
        }
    }
}

#endif // _CV_MAT_SERIALIZER
