/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  Copyright (c) 2016, Guan-Horng Liu.
*  All rights reserved.
*********************************************************************/
// NOTE This source code is fetched from pyReedsShepp https://github.com/ghliu/pyReedsShepp
// The original source code for reeds-shepp can be found in ompl https://github.com/ompl/ompl
// Then the ompl code is fetched and modified in pyReedsShepp https://github.com/ghliu/pyReedsShepp

#ifndef SPACES_REEDS_SHEPP_STATE_SPACE_
#define SPACES_REEDS_SHEPP_STATE_SPACE_

//#include <boost/math/constants/constants.hpp>
#include <cassert>
#include <cmath>
#include <limits>
#include <algorithm>

typedef int (*ReedsSheppPathSamplingCallback)(int idx, double q[3], void* user_data);
typedef int (*ReedsSheppPathTypeCallback)(int t, void* user_data);

class ReedsSheppStateSpace
{
public:

    /** \brief The Reeds-Shepp path segment types */
    enum ReedsSheppPathSegmentType { RS_NOP=0, RS_LEFT=1, RS_STRAIGHT=2, RS_RIGHT=3 };

    /** \brief Reeds-Shepp path types */
    static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
    
    /** \brief Complete description of a ReedsShepp path */
    class ReedsSheppPath
    {
    public:
        ReedsSheppPath(const ReedsSheppPathSegmentType* type=reedsSheppPathType[0],
            double t=std::numeric_limits<double>::max(), double u=0., double v=0.,
            double w=0., double x=0.);
        
        double length() const { return totalLength_; }

        /** Path segment types */
        const ReedsSheppPathSegmentType* type_;
        /** Path segment lengths */
        double length_[5];
        /** Total length */
        double totalLength_;
    };

    ReedsSheppStateSpace(double turningRadius) : rho_(turningRadius) {}

    double distance(double q0[3], double q1[3]);

    void type(double q0[3], double q1[3], ReedsSheppPathTypeCallback cb, void* user_data);

    void sample(double q0[3], double q1[3], double step_size, ReedsSheppPathSamplingCallback cb, double* arr);

    /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
    ReedsSheppPath reedsShepp(double q0[3], double q1[3]);

    void interpolate(double q0[3], ReedsSheppPath &path, double seg, double q[3]);

    /** \brief Turning radius */
    double rho_;
};

#endif
