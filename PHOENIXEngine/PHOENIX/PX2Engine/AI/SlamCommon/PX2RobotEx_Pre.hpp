// PX2RobotEx_Pre.hpp

#ifndef PX2ROBOTEX_PRE_HPP
#define PX2ROBOTEX_PRE_HPP

// this file is in AI Path,but shoud putted in slampath to persurt quick compile in visual studio

#include "PX2CorePre.hpp"

#if defined(SLAM_DLL_EXPORT)
#define SLAM_DLL_ITEM __declspec(dllexport)
#elif defined(SLAM_DLL_IMPORT)
#define SLAM_DLL_ITEM __declspec(dllimport)
#else
#define SLAM_DLL_ITEM
#endif

#endif