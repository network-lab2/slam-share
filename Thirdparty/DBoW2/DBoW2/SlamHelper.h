/**
 * File: SLAM_HELPER.h
 * Date: January 2022
 * Author: 
 * Description: bag of words vector shim for SLAM
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_SLAM_HELPER__
#define __D_T_SLAM_HELPER__

#include <iostream>
#include <map>
#include <vector>

#include <boost/serialization/serialization.hpp>
#include <boost/interprocess/containers/map.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

namespace DBoW2 {


typedef std::pair<const unsigned int, double> ValueType_bow;
typedef boost::interprocess::allocator<ValueType_bow,boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator_bow;
typedef boost::interprocess::map<unsigned int,double,std::less<unsigned int >,ShmemAllocator_bow> bowMap;

}  // namespace DBoW2
#endif