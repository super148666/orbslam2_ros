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

#ifndef MAP_H
#define MAP_H

#include "KeyFrame.h"
#include "MapPoint.h"
#include <set>

#include <mutex>

#include <boost/serialization/split_member.hpp>
#include "cvmat_serialization.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

namespace ORB_SLAM2 {
    class KeyFrame;

    class MapPoint;

    class Map {
        friend class boost::serialization::access;

        // XXX: Need to modify code for ORB_SLAM2
        void save(boost::archive::binary_oarchive &ar, const unsigned int version) const;

        // XXX: Need to modify code for ORB_SLAM2
        void load(boost::archive::binary_iarchive &ar, const unsigned int version);

        BOOST_SERIALIZATION_SPLIT_MEMBER()

    public:
        Map();

        void AddKeyFrame(KeyFrame *pKF);

        void AddMapPoint(MapPoint *pMP);

        void EraseMapPoint(MapPoint *pMP);

        void EraseKeyFrame(KeyFrame *pKF);

        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

        void InformNewBigChange();

        int GetLastBigChangeIdx();

        std::vector<KeyFrame *> GetAllKeyFrames();

        std::vector<MapPoint *> GetAllMapPoints();

        std::vector<MapPoint *> GetReferenceMapPoints();

        long unsigned int MapPointsInMap();

        long unsigned int KeyFramesInMap();

        long unsigned int GetMaxKFid();

        void clear();

        vector<KeyFrame *> mvpKeyFrameOrigins;

        std::mutex mMutexMapUpdate;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::mutex mMutexPointCreation;

    protected:
        std::set<MapPoint *> mspMapPoints;
        std::set<KeyFrame *> mspKeyFrames;

        std::vector<MapPoint *> mvpReferenceMapPoints;

        long unsigned int mnMaxKFid;

        // Index related to a big change in the map (loop closure, global BA)
        int mnBigChangeIdx;

        std::mutex mMutexMap;
    };

} //namespace ORB_SLAM

#endif // MAP_H
