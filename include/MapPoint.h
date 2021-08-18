/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"


#include<opencv2/core/core.hpp>
#include<mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class Frame;

class MapPoint
{

public:
    MapPoint();

    MapPoint(const cv::Mat &Pos, boost::interprocess::offset_ptr<KeyFrame>  pRefKF, boost::interprocess::offset_ptr<Map>  pMap);
    MapPoint(const double invDepth, cv::Point2f uv_init, boost::interprocess::offset_ptr<KeyFrame>  pRefKF, boost::interprocess::offset_ptr<KeyFrame>  pHostKF, boost::interprocess::offset_ptr<Map>  pMap);
    MapPoint(const cv::Mat &Pos,  boost::interprocess::offset_ptr<Map>  pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);

    cv::Mat GetWorldPos();

    cv::Mat GetNormal();

    cv::Matx31f GetWorldPos2();

    cv::Matx31f GetNormal2();

    boost::interprocess::offset_ptr<KeyFrame>  GetReferenceKeyFrame();

    std::map<boost::interprocess::offset_ptr<KeyFrame> ,std::tuple<int,int>> GetObservations();
    int Observations();

    void AddObservation(boost::interprocess::offset_ptr<KeyFrame>  pKF,int idx);
    void EraseObservation(boost::interprocess::offset_ptr<KeyFrame>  pKF);

    std::tuple<int,int> GetIndexInKeyFrame(boost::interprocess::offset_ptr<KeyFrame>  pKF);
    bool IsInKeyFrame(boost::interprocess::offset_ptr<KeyFrame>  pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(boost::interprocess::offset_ptr<MapPoint>  pMP);    
    boost::interprocess::offset_ptr<MapPoint>  GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();
    void SetNormalVector(cv::Mat& normal);

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, boost::interprocess::offset_ptr<KeyFrame> pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    boost::interprocess::offset_ptr<Map>  GetMap();
    void UpdateMap(boost::interprocess::offset_ptr<Map>  pMap);



public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackDepth;
    float mTrackDepthR;
    float mTrackProjXR;
    float mTrackProjYR;
    bool mbTrackInView, mbTrackInViewR;
    int mnTrackScaleLevel, mnTrackScaleLevelR;
    float mTrackViewCos, mTrackViewCosR;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;

    // Variable used by merging
    cv::Mat mPosMerge;
    cv::Mat mNormalVectorMerge;


    // Fopr inverse depth optimization
    double mInvDepth;
    double mInitU;
    double mInitV;
    boost::interprocess::offset_ptr<KeyFrame>  mpHostKF;

    static std::mutex mGlobalMutex;

    unsigned int mnOriginMapId;


    // all the matrix variable addresses.
    char *mPosGBA_ptr;
    char *mPosMerge_ptr;
    char *mNormalVectorMerge_ptr;

    // For the absolute coordinates
    char *mWorldPos_ptr;
    char *mWorldPosx_ptr;

    char *mNormalVector_ptr;
    char *mNormalVectorx_ptr;
    char *mDescriptor_ptr;



protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;
     cv::Matx31f mWorldPosx;

     // Keyframes observing the point and associated index in keyframe
     std::map<boost::interprocess::offset_ptr<KeyFrame> ,std::tuple<int,int> > mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;
     cv::Matx31f mNormalVectorx;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     boost::interprocess::offset_ptr<KeyFrame>  mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     boost::interprocess::offset_ptr<MapPoint>  mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     boost::interprocess::offset_ptr<Map>  mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
     std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
