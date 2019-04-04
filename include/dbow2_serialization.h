#ifndef _DBOW2_SERIALIZER
#define _DBOW2_SERIALIZER

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>

BOOST_SERIALIZATION_SPLIT_FREE(DBoW2::BowVector)
namespace boost {
    namespace serialization {

        /** Serialization support for cv::Mat */
        template<class Archive>
        void save(Archive &ar, const DBoW2::BowVector &in, const unsigned int version) {
            auto sz = in.size();
            ar & sz;
            for (auto &item : in) {
                ar & item.first & item.second;
            }
        }

        /** Serialization support for cv::Mat */
        template<class Archive>
        void load(Archive &ar, DBoW2::BowVector &out, const unsigned int version) {
//        cout<<"loading DBoW2::BowVector ...\n";
            out.clear();
            size_t sz;
            ar & sz;
            for (size_t i = 0; i < sz; i++) {
                DBoW2::WordId key;
                DBoW2::WordValue value;
                ar & key;
                ar & value;
                out[key] = value;
            }
//        cout<<"DONE loading DBoW2::BowVector ...\n";
        }

    }
}


BOOST_SERIALIZATION_SPLIT_FREE(DBoW2::FeatureVector)
namespace boost {
    namespace serialization {

        template<class Archive>
        void save(Archive &ar, const DBoW2::FeatureVector &in, const unsigned int version) {
            auto sz = in.size();
            ar & sz;
            for (auto &item : in) {
                ar & item.first & item.second;
            }
        }

        template<class Archive>
        void load(Archive &ar, DBoW2::FeatureVector &out, const unsigned int version) {
//            cout<<"loading DBoW2::FeatureVector ...\n";
            out.clear();
            size_t sz;
            ar & sz;
            for (size_t i = 0; i < sz; i++) {
                DBoW2::NodeId key;
                std::vector<unsigned int> value;
                ar & key;
                ar & value;
                out[key] = value;
            }
//            cout<<"DONE loading DBoW2::FeatureVector ...\n";
        }
    }
}


#endif // _DBOW2_SERIALIZER
