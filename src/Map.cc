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

#include "Map.h"


namespace ORB_SLAM2 {

    Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0) {
    }

    void Map::AddKeyFrame(KeyFrame *pKF) {
        cout<<"map locked\n";
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.insert(pKF);
        if (pKF->mnId > mnMaxKFid)
            mnMaxKFid = pKF->mnId;
        cout<<"map unlocked\n";
    }

    void Map::AddMapPoint(MapPoint *pMP) {
        cout<<"map locked\n";
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
        cout<<"map unlocked\n";
    }

    void Map::EraseMapPoint(MapPoint *pMP) {
        cout<<"map locked\n";
        try{
            unique_lock<mutex> lock(mMutexMap);
        }
        catch (const std::exception& e){
            cout<<e.what()<<"\n";
        }

        mspMapPoints.erase(pMP);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
        cout<<"map unlocked\n";
    }

    void Map::EraseKeyFrame(KeyFrame *pKF) {
        cout<<"map locked\n";
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
        cout<<"map unlocked\n";
    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs) {
        cout<<"map locked\n";
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
        cout<<"map unlocked\n";
    }

    void Map::InformNewBigChange() {
        cout<<"map locked\n";
        unique_lock<mutex> lock(mMutexMap);
        mnBigChangeIdx++;
        cout<<"map unlocked\n";
    }

    int Map::GetLastBigChangeIdx() {
        cout<<"map locked\n";
        unique_lock<mutex> lock(mMutexMap);
        cout<<"map unlocked\n";
        return mnBigChangeIdx;
    }

    vector<KeyFrame *> Map::GetAllKeyFrames() {
        cout<<"map locked\n";
        unique_lock<mutex> lock(mMutexMap);
        cout<<"map unlocked\n";
        return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
    }

    vector<MapPoint *> Map::GetAllMapPoints() {
        cout<<"map locked\n";
        unique_lock<mutex> lock(mMutexMap);
        cout<<"map unlocked\n";
        return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
    }

    long unsigned int Map::MapPointsInMap() {
        cout<<"map locked\n";
        unique_lock<mutex> lock(mMutexMap);
        cout<<"map unlocked\n";
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap() {
        cout<<"map locked\n";
        unique_lock<mutex> lock(mMutexMap);
        cout<<"map unlocked\n";
        return mspKeyFrames.size();
    }

    vector<MapPoint *> Map::GetReferenceMapPoints() {
        cout<<"map locked\n";
        unique_lock<mutex> lock(mMutexMap);
        cout<<"map unlocked\n";
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetMaxKFid() {
        cout<<"map locked\n";
        unique_lock<mutex> lock(mMutexMap);
        cout<<"map unlocked\n";
        return mnMaxKFid;
    }

    void Map::clear() {
        for (set<MapPoint *>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++)
            delete *sit;

        for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
            delete *sit;

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    void Map::save(boost::archive::binary_oarchive &ar, const unsigned int version) const {

        auto numKeyFrames = mspKeyFrames.size();
        auto numMapPoints = mspMapPoints.size();
        ar & mnMaxKFid & mnBigChangeIdx;

        // save num of keyframe
        ar & numKeyFrames;

        // save all keyframes
        for (auto &kf:mspKeyFrames) {
            ar & kf;
        }

        // save mvpKeyFrameOrigins with just id reference
        auto imvpKeyFrameOrigins = createIdList(mvpKeyFrameOrigins);
        ar & imvpKeyFrameOrigins;

        // save num of map points
        ar & numMapPoints;

        // save all map points
        for (auto &mp:mspMapPoints) {
            ar & mp;
        }

        // save mvpReferenceMapPoints
        auto imvpReferenceMapPoints = createIdList(mvpReferenceMapPoints);
        ar & imvpReferenceMapPoints;

    }

    void Map::load(boost::archive::binary_iarchive &ar, const unsigned int version) {
        {
            cout << "loading Map ...\n";
            cout<<"map locked\n";
            unique_lock<mutex> lock(mMutexMap);
            ar & mnMaxKFid & mnBigChangeIdx;

            // load num of keyframes
            uint64_t numKeyFrames;
            ar & numKeyFrames;


            // clear objectListLookup in KeyFrame
            KeyFrame::objectListLookup.clear();
            mspKeyFrames.clear();

            for (uint64_t id = 0; id < numKeyFrames; id++) {
                auto kf = new KeyFrame;
                ar & kf;
                mspKeyFrames.insert(kf);
                KeyFrame::objectListLookup[kf->mnId] = kf;
            }

            // load mvpKeyFrameOrigins
            vector<uint64_t> imvpKeyFrameOrigins;
            ar & imvpKeyFrameOrigins;
            mvpKeyFrameOrigins = createObjectList<KeyFrame>(imvpKeyFrameOrigins);

            // load num of map points
            long unsigned int numMapPoints;
            ar & numMapPoints;

            // clear objectListLookup in MapPoint
            MapPoint::objectListLookup.clear();
            mspMapPoints.clear();

            for (uint64_t id = 0; id < numMapPoints; id++) {
                auto mp = new MapPoint;
                ar & mp;
                mspMapPoints.insert(mp);
                MapPoint::objectListLookup[mp->mnId] = mp;
            }


            for (auto&kf:mspKeyFrames) {
                kf->Recover();
            }

            for (auto&mp:mspMapPoints) {
                mp->Recover();
                mp->SetMap(this);
            }

            // load mvpReferenceMapPoints
            vector<uint64_t> imvpReferenceMapPoints;
            ar & imvpReferenceMapPoints;
            mvpReferenceMapPoints = createObjectList<MapPoint>(imvpReferenceMapPoints);
            cout<<"map unlocked\n";
            cout << "DONE loading Map ...\n";

        }
    }

} //namespace ORB_SLAM
