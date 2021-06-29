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


#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include <math.h>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

namespace ORB_SLAM3
{

class LoopClosing;

class Optimizer
{
public:

    void static BundleAdjustment(const std::vector<boost::interprocess::offset_ptr<KeyFrame> > &vpKF, const std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(boost::interprocess::offset_ptr<Map>  pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static FullInertialBA(boost::interprocess::offset_ptr<Map> pMap, int its, const bool bFixLocal=false, const unsigned long nLoopKF=0, bool *pbStopFlag=NULL, bool bInit=false, float priorG = 1e2, float priorA=1e6, Eigen::VectorXd *vSingVal = NULL, bool *bHess=NULL);

    void static LocalBundleAdjustment(boost::interprocess::offset_ptr<KeyFrame>  pKF, bool *pbStopFlag, vector<boost::interprocess::offset_ptr<KeyFrame> > &vpNonEnoughOptKFs);
    void static LocalBundleAdjustment(boost::interprocess::offset_ptr<KeyFrame>  pKF, bool *pbStopFlag, boost::interprocess::offset_ptr<Map> pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges);

    void static MergeBundleAdjustmentVisual(boost::interprocess::offset_ptr<KeyFrame>  pCurrentKF, vector<boost::interprocess::offset_ptr<KeyFrame> > vpWeldingKFs, vector<boost::interprocess::offset_ptr<KeyFrame> > vpFixedKFs, bool *pbStopFlag);

    int static PoseOptimization(Frame* pFrame);

    int static PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit = false);
    int static PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit = false);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(boost::interprocess::offset_ptr<Map>  pMap, boost::interprocess::offset_ptr<KeyFrame>  pLoopKF, boost::interprocess::offset_ptr<KeyFrame>  pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<boost::interprocess::offset_ptr<KeyFrame> , set<boost::interprocess::offset_ptr<KeyFrame> > > &LoopConnections,
                                       const bool &bFixScale);
    void static OptimizeEssentialGraph6DoF(boost::interprocess::offset_ptr<KeyFrame>  pCurKF, vector<boost::interprocess::offset_ptr<KeyFrame> > &vpFixedKFs, vector<boost::interprocess::offset_ptr<KeyFrame> > &vpFixedCorrectedKFs,
                                           vector<boost::interprocess::offset_ptr<KeyFrame> > &vpNonFixedKFs, vector<boost::interprocess::offset_ptr<MapPoint> > &vpNonCorrectedMPs, double scale);
    void static OptimizeEssentialGraph(boost::interprocess::offset_ptr<KeyFrame>  pCurKF, vector<boost::interprocess::offset_ptr<KeyFrame> > &vpFixedKFs, vector<boost::interprocess::offset_ptr<KeyFrame> > &vpFixedCorrectedKFs,
                                       vector<boost::interprocess::offset_ptr<KeyFrame> > &vpNonFixedKFs, vector<boost::interprocess::offset_ptr<MapPoint> > &vpNonCorrectedMPs);
    void static OptimizeEssentialGraph(boost::interprocess::offset_ptr<KeyFrame>  pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3);
    // For inetial loopclosing
    void static OptimizeEssentialGraph4DoF(boost::interprocess::offset_ptr<Map>  pMap, boost::interprocess::offset_ptr<KeyFrame>  pLoopKF, boost::interprocess::offset_ptr<KeyFrame>  pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<boost::interprocess::offset_ptr<KeyFrame> , set<boost::interprocess::offset_ptr<KeyFrame> > > &LoopConnections);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono) (OLD)
    static int OptimizeSim3(boost::interprocess::offset_ptr<KeyFrame>  pKF1, boost::interprocess::offset_ptr<KeyFrame>  pKF2, std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono) (NEW)
    static int OptimizeSim3(boost::interprocess::offset_ptr<KeyFrame>  pKF1, boost::interprocess::offset_ptr<KeyFrame>  pKF2, std::vector<boost::interprocess::offset_ptr<MapPoint> > &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale,
                            Eigen::Matrix<double,7,7> &mAcumHessian, const bool bAllPoints=false);
    static int OptimizeSim3(boost::interprocess::offset_ptr<KeyFrame> pKF1, boost::interprocess::offset_ptr<KeyFrame> pKF2, vector<boost::interprocess::offset_ptr<MapPoint> > &vpMatches1, vector<boost::interprocess::offset_ptr<KeyFrame> > &vpMatches1KF,
                     g2o::Sim3 &g2oS12, const float th2, const bool bFixScale, Eigen::Matrix<double,7,7> &mAcumHessian,
                     const bool bAllPoints = false);

    // For inertial systems

    void static LocalInertialBA(boost::interprocess::offset_ptr<KeyFrame>  pKF, bool *pbStopFlag, boost::interprocess::offset_ptr<Map> pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge = false, bool bRecInit = false);

    void static MergeInertialBA(boost::interprocess::offset_ptr<KeyFrame>  pCurrKF, boost::interprocess::offset_ptr<KeyFrame>  pMergeKF, bool *pbStopFlag, boost::interprocess::offset_ptr<Map> pMap, LoopClosing::KeyFrameAndPose &corrPoses);

    // Local BA in welding area when two maps are merged
    void static LocalBundleAdjustment(boost::interprocess::offset_ptr<KeyFrame>  pMainKF,vector<boost::interprocess::offset_ptr<KeyFrame> > vpAdjustKF, vector<boost::interprocess::offset_ptr<KeyFrame> > vpFixedKF, bool *pbStopFlag);

    // Marginalize block element (start:end,start:end). Perform Schur complement.
    // Marginalized elements are filled with zeros.
    static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end);
    // Condition block element (start:end,start:end). Fill with zeros.
    static Eigen::MatrixXd Condition(const Eigen::MatrixXd &H, const int &start, const int &end);
    // Remove link between element 1 and 2. Given elements 1,2 and 3 must define the whole matrix.
    static Eigen::MatrixXd Sparsify(const Eigen::MatrixXd &H, const int &start1, const int &end1, const int &start2, const int &end2);

    // Inertial pose-graph
    void static InertialOptimization(boost::interprocess::offset_ptr<Map> pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel=false, bool bGauss=false, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(boost::interprocess::offset_ptr<Map> pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(vector<boost::interprocess::offset_ptr<KeyFrame> > vpKFs, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(boost::interprocess::offset_ptr<Map> pMap, Eigen::Matrix3d &Rwg, double &scale);
};

} //namespace ORB_SLAM3

#endif // OPTIMIZER_H
