/*
 * MapObjectSerialization.h
 *
 *  Created on: Nov 15, 2015
 *      Author: sujiwo
 */

#ifndef INCLUDE_MAPOBJECTSERIALIZATION_H_
#define INCLUDE_MAPOBJECTSERIALIZATION_H_


#include <set>
#include <vector>
#include <list>
#include <map>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/map.hpp>
#include "cvmat_serialization.h"
#include <boost/serialization/base_object.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include "KeyFrame.h"
#include "MapPoint.h"
#include "KeyFrameDatabase.h"


using std::set;
using std::vector;
using std::map;
using std::list;
using namespace ORB_SLAM2;
using std::out_of_range;


// comment this if you don't want to save images along with keyframes
//#define MAP_SAVE_IMAGE 1


/*
 * createIdList functions convert list of map objects (in list, set, vector and map) into list of ID
 * createObjectList functions do their reverse
 */

template<typename MapObject>
set<uint64_t> createIdList(const set<MapObject *> &mapObjectList) {
    typedef typename set<MapObject *>::const_iterator ciMapObj;
    set<uint64_t> idList;

    for (ciMapObj it = mapObjectList.begin(); it != mapObjectList.end(); it++) {
        MapObject *obj = *it;
        if (obj == NULL) continue;
        idList.insert(obj->mnId);
    }

    return idList;
}


template<typename MapObject>
set<MapObject *> createObjectList(const set<uint64_t> &mapIdList) {
    set<MapObject *> objSet;

    for (set<uint64_t>::const_iterator it = mapIdList.begin(); it != mapIdList.end(); it++) {
        try {
            MapObject *obj = MapObject::objectListLookup.at(*it);
            objSet.insert(obj);
        } catch (out_of_range &e) {
            continue;
        }
    }

    return objSet;
}


template<typename MapObject>
vector<uint64_t> createIdList(const vector<MapObject *> &mapObjectList) {
    vector<uint64_t> idList = vector<uint64_t>(mapObjectList.size(), static_cast<uint64_t> (-1));

    int p = 0;
    for (typename vector<MapObject *>::const_iterator it = mapObjectList.begin(); it != mapObjectList.end(); it++) {
        MapObject *obj = *it;
        if (obj != NULL)
            idList[p] = obj->mnId;
        p++;
    }

    return idList;
}


template<typename MapObject>
vector<MapObject *> createObjectList(const vector<uint64_t> &mapIdList) {
    vector<MapObject *> objVect = vector<MapObject *>(mapIdList.size(), static_cast<MapObject *> (NULL));

    int p = 0;
    for (vector<uint64_t>::const_iterator it = mapIdList.begin(); it != mapIdList.end(); it++) {
        MapObject *obj;
        try {
            obj = MapObject::objectListLookup.at(*it);
        } catch (out_of_range &e) {
            obj = NULL;
        }
        objVect[p] = obj;
        p++;
    }

    return objVect;
}


template<typename MapObject>
vector<MapObject *> purgeNull(const vector<MapObject *> &mapObjectList) {
    vector<MapObject *> newObjectVector;

    for (typename vector<MapObject *>::const_iterator it = mapObjectList.begin(); it != mapObjectList.end(); it++) {
        MapObject *obj = *it;
        if (obj == NULL)
            continue;
        newObjectVector.push_back(obj);
    }

    return newObjectVector;
}


template<typename MapObject>
list<uint64_t> createIdList(const list<MapObject *> &mapObjectList) {
    typedef typename list<MapObject *>::const_iterator ciMapObj;
    list<uint64_t> idList;

    for (ciMapObj it = mapObjectList.begin(); it != mapObjectList.end(); it++) {
        MapObject *obj = *it;
        if (obj == NULL) continue;
        idList.push_back(obj->mnId);
    }

    return idList;
}


template<typename MapObject>
list<MapObject *> createObjectList(const list<uint64_t> &mapIdList) {
    list<MapObject *> objVect;

    for (list<uint64_t>::const_iterator it = mapIdList.begin(); it != mapIdList.end(); it++) {
        try {
            MapObject *obj = MapObject::objectListLookup.at(*it);
            objVect.push_back(obj);
        } catch (out_of_range &e) {
            continue;
        }
    }

    return objVect;
}


template<typename MapObject, typename W>
map<uint64_t, W> createIdList(const map<MapObject *, W> &mapObjectList) {
    typedef typename map<MapObject *, W>::const_iterator ciMapObj;
    map<uint64_t, W> idList;

    for (ciMapObj it = mapObjectList.begin(); it != mapObjectList.end(); it++) {
        MapObject *obj = it->first;
        if (obj == NULL) continue;
        idList[obj->mnId] = it->second;
    }

    return idList;
}


template<typename MapObject, typename W>
map<MapObject *, W> createObjectList(const map<uint64_t, W> &mapIdList) {
    map<MapObject *, W> objMap;
    typedef typename map<uint64_t, W>::const_iterator ci_t;

    for (ci_t it = mapIdList.begin(); it != mapIdList.end(); it++) {
        try {
            MapObject *obj = MapObject::objectListLookup.at(it->first);
            if (obj == NULL)
                continue;
            objMap[obj] = it->second;
        } catch (out_of_range &e) {
            continue;
        }
    }
    return objMap;
}


template<typename T>
bool debugSerialization(const T &src) {
    stringstream ssOut;
    boost::archive::binary_oarchive outarc(ssOut);
    outarc << src;
    string outStr = ssOut.str();

    string outStrX = outStr;
    stringstream ssIn(outStr);
    boost::archive::binary_iarchive inarc(ssIn);
    T tgt;
    inarc >> tgt;

    return true;
}


void debugList(map<uint64_t, int> &targetList, const char *filename);


#endif /* INCLUDE_MAPOBJECTSERIALIZATION_H_ */
