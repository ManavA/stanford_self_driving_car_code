/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/

#ifndef TCGALTYPES_
#define TCGALTYPES_

#include <CGAL/Cartesian.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Point_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Direction_2.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Object.h>
#include <list>
#include <limits>
#include <cassert>
#include <math.h>

namespace CGAL_Geometry
{

using namespace std;

//! The used Kernel for CGAL (all geometric calculations).
typedef CGAL::Simple_cartesian < double > Kernel;

/*!
   The Point type (all points have this type)
 * @deprecated only for old code. Use Point_2 instead.
 */
typedef CGAL::Point_2< Kernel > Point;

//! The Point type (all points have this type)
typedef CGAL::Point_2< Kernel > Point_2;

//! The 2D Polygon (all polygons have this type)
typedef CGAL::Polygon_2< Kernel, std::vector<Point_2>  > Polygon_2;

//! The 2D Vector (the type of the vectors)
typedef CGAL::Vector_2 < Kernel > Vector_2;

//! The 2D Direction
typedef CGAL::Direction_2< Kernel > Direction_2;

//! The 2D Line
typedef CGAL::Line_2< Kernel > Line_2;

//! The 2D Segment
typedef CGAL::Segment_2< Kernel > Segment_2;

//! The 2D Vertex_iterator
typedef Polygon_2::Vertex_iterator Polygon_vertex_iterator;

//! The 2D Direction
typedef CGAL::Direction_2< Kernel > Direction_2;

//! The 2D Aff_transformation_2
typedef CGAL::Aff_transformation_2< Kernel >  Transformation;

//! The 2D Circle
typedef CGAL::Circle_2< Kernel > Circle_2;

//! The 2D Box
typedef CGAL::Bbox_2 Bbox_2;


/*!
   An extended 2D Vector/Point
 *
 * has an added method normalize that returns a normalized Vector_2.
 * Because this class does not add any member variables it is interchangeable
 * with CGAL::Vector_2 < Kernel >.
 */
template < class CLASS >
struct VectorPoint_extended : public CLASS
{
    typedef CLASS type;
    static VectorPoint_extended& cast(CLASS& v)
    {
        return *reinterpret_cast< VectorPoint_extended* >(&v);
    };
    /**
     *
     * @return Returns a normalized Vector_2 of this Vector_2_extended
     */
    static CLASS normalize (const CLASS& value)
    {
        Kernel::FT norm = (value.x()*value.x())+(value.y()*value.y());
        /* Better to test for (near-)zero norm before taking reciprocal. */
        Kernel::FT scl;
        if (norm < delta) // norm can only be posisitve
        {
            std::cout << "norm very small!" << std::endl;
            scl = 1.0/(std::sqrt(delta));
        }
        else
        {
            scl = 1.0/( std::sqrt(norm) );
        }
        return CLASS(value.x()*scl, value.y()*scl);
    };
    static bool isNearNull(const CLASS& value)
    {
        return (dabs(value.x()) < delta && dabs(value.y()) < delta);
    };
    static bool isDegenerated(const CLASS& value)
    {
        return isdegenerated(value.x()) || isdegenerated(value.y());

        //return value.x()==0.0/0.0 || value.y()==0.0/0.0;
    };
private:
    static const Kernel::FT delta = 0.0000000001;
};


typedef VectorPoint_extended< CGAL::Vector_2 < Kernel > > Vector_2_extended;
typedef VectorPoint_extended< CGAL::Point_2 < Kernel > > Point_2_extended;


inline bool isdegenerated(const Kernel::FT v) {
	return ! isfinite( v );
	//    return v==INFINITY || v==-INFINITY || isnan((float)v);
};

/**
 * A long double abs(long double) seems to be missing. So we implemente it here.
 * @param value the value to enforce to be positive
 * @return if value is negative returns -value otherwise value
 */
inline Kernel::FT dabs(const Kernel::FT& value)
{
    if (value < 0.0)
    {
        return -value;
    }
    else
    {
        return value;
    }
};

/**
 * reimplement standard sqrt function for debugging purpose.
 * @param r
 * @return
 */
inline Kernel::FT sqrt_checked(Kernel::FT r)
{
    assert(!isdegenerated(r));
    assert(r >= 0.0);
    return std::sqrt (r);
};

// the template function assertValidity ensures that
// the members of CGAL objects have valid values (not NaN or INFINITY)
// the function has specialisations for
// * double
// * Point_2
// * Vector_2
// * Line_2
// * Segment_2
template <typename CLASS> inline void assertValidity(const CLASS value)
{
    assert(!VectorPoint_extended<CLASS>::isDegenerated(value));
};

template <> inline void assertValidity<Kernel::FT>(const Kernel::FT value)
{
    assert(!isdegenerated(value));
};
template <> inline void assertValidity<Line_2>(const Line_2 value)
{
    assertValidity(value.a());
    assertValidity(value.b());
    assertValidity(value.c());
};
template <> inline void assertValidity<Segment_2>(const Segment_2 value)
{
    assertValidity(value.source());
    assertValidity(value.target());
};


inline double theta_between_minuspi_and_pi(double theta)
{
    int multiplier;

    if (theta >= -M_PI && theta < M_PI)
        return theta;

    multiplier = (int)(theta / (2*M_PI));
    theta = theta - multiplier*2*M_PI;
    if (theta >= M_PI)
        theta -= 2*M_PI;
    if (theta < -M_PI)
        theta += 2*M_PI;

    return theta;
}


#define M_2PI (M_PI * 2.)


//*****************************************************************************
//
//		Math helper functions
//
//*****************************************************************************

//-----------------------------------------------------------------------------
//		sqr()
//-----------------------------------------------------------------------------

//! caluclates the sqare (x^2) of a value \a x
template <typename float_t>
inline float_t
sqr(const float_t& x)
{
	return x*x;
}

//-----------------------------------------------------------------------------
//		normAngle()
//-----------------------------------------------------------------------------

//! returns the corresponding angle out of [0,2Pi]
template <typename float_t>
inline float_t
normAngle(const float_t& angle)
{
	return angle - floor(angle * M_1_PI * 0.5) * M_2PI;
}

//-----------------------------------------------------------------------------
//		deltaAngle()
//-----------------------------------------------------------------------------

//! returns the minimal absolut delta between the two angles out of [0,Pi]
template <typename float_t>
inline float_t
deltaAngle(const float_t& angle1, const float_t& angle2)
{
	float_t delta = normAngle(angle2 - angle1);
	return delta > M_PI ? M_2PI - delta : delta;
}

//-----------------------------------------------------------------------------
//		normPeriod()
//-----------------------------------------------------------------------------

//! returns the corresponding modulo value out of [0,period]
template <typename float_t>
inline float_t
normPeriod(const float_t& value, const float_t& period)
{
	return value - floor(value / period) * period;
}

//! returns the corresponding modulo value out of [0,period]
template <typename float_t>
inline float_t
normPeriod(const float_t& value, const float_t& period, const float_t& epsilon)
{
	float_t new_val = value - floor(value / period) * period;
	return (period - value <= epsilon ? 0. : new_val);
}


//*****************************************************************************
//
//		Geometry functions
//
//*****************************************************************************

//-----------------------------------------------------------------------------
//		rotate()
//-----------------------------------------------------------------------------

//! rotate a vector with \a angle
template <typename float_t>
inline typename CGAL::Cartesian<float_t>::Vector_2
rotate(const typename CGAL::Cartesian<float_t>::Vector_2& vec, const float_t& angle)
{
	float_t sinv = sin(angle);
	float_t cosv = cos(angle);
	return typename CGAL::Cartesian<float_t>::Vector_2(vec.x()*cosv - vec.y()*sinv, vec.x()*sinv + vec.y()*cosv);
}

//-----------------------------------------------------------------------------
//		length()
//-----------------------------------------------------------------------------

//! calculates the length of a vector
template <typename float_t>
inline float_t
length(const typename CGAL::Cartesian<float_t>::Vector_2& vec)
{
	return sqrt(squared_length(vec));
}

//-----------------------------------------------------------------------------
//		distance()
//-----------------------------------------------------------------------------

//! calculates the euclidian distance between two points
template <typename float_t>
inline float_t
//distance(const typename Cartesian<float_t>::Point_2& p1, const typename Cartesian<float_t>::Point_2& p2)
distance(const CGAL::Point_2< CGAL::Cartesian<float_t> >& p1, const CGAL::Point_2< CGAL::Cartesian<float_t> >& p2)
{
	return std::sqrt( (p2-p1).squared_length() );
}

//-----------------------------------------------------------------------------
//		angle()
//-----------------------------------------------------------------------------

//! calculates the angle of vector
template <typename float_t>
inline float_t
angle(const CGAL::Vector_2< CGAL::Cartesian<float_t> >& vec)
{
	return atan2(vec.y(), vec.x());
}

inline double angle(const Vector_2& vec)
{
	return atan2(vec.y(), vec.x());
}


};

#endif              /*TCGALTYPES_ */
