/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#pragma once

/**
 * @file
 * @author Jocelyn Meyron (\c jocelyn.meyron@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2020/12/04
 *
 * @author Tristan Roussilllon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2024/09/16
 *
 * Helper functions for plane-probing algorithms.
 *
 * This file is part of the DGtal library.
 */

#if defined(PlaneProbingEstimatorHelper_RECURSES)
#error Recursive header files inclusion detected in PlaneProbingEstimatorHelper.h
#else // defined(PlaneProbingEstimatorHelper_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PlaneProbingEstimatorHelper_RECURSES

#if !defined PlaneProbingEstimatorHelper_h
/** Prevents repeated inclusion of headers. */
#define PlaneProbingEstimatorHelper_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <array>
#include <cassert>
#include "DGtal/math/linalg/SimpleMatrix.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
    namespace detail
    {
        /**
         * Version of DGtal::PointVector::squaredNorm with no conversion to double.
         *
         * @param aPoint an input digital point.
         * @return the squared norm of the input point.
         */
        template < typename Point >
        typename Point::Coordinate squaredNorm (Point const& aPoint);

        /**
         * Determinant of a NxN matrix represented by a two-dimensional static array.
         *
         * @param aMatrix the static array representing the matrix.
         * @return the determinant of the input matrix.
         */
        template < int N, typename T >
        T determinant (const T aMatrix[N][N]);

        /**
         * Computes the distance of a point to a sphere passing through 4 given points.
         *
         * @param aPoints 5 points where the first 4 define the sphere and the last one is the point to which we want to compute the distance.
         * @return the distance of the last point to the sphere defined by the first 4 points.
        */
        template < typename Point >
        inline
        typename Point::Coordinate distToSphere (std::array<Point, 5> const& aPoints);

        /**
         * Test if a pair of vectors form a reduced basis.
         *
         * @param aU the first vector.
         * @param aV the second vector.
         * @return 'true' if (\a aU, \a aV) is a reduced basis, 'false' otherwise.
         */
        template < typename Point >
        inline
        bool isBasisReduced (Point const& aU, Point const& aV);

        /////////////////////////////////////////////////////////////////////////////
        // template class PointOnProbingRay
        /**
         * Description of template class 'PointOnProbingRay' <p>
         * \brief A ray consists of a permutation \f$ \sigma \f$ and an integer index \f$ \lambda \f$ (position on the ray).
         * For a triplet of vectors \f$ (m_k)_{0 \leq k \leq 2} \f$ and a point \f$ q \f$, a point on the ray is defined as:
         * \f$ q - m_{\sigma(0)} + m_{\sigma(1)} + \lambda m_{\sigma(2)} \f$. \f$ q - m_{\sigma(0)} + m_{\sigma(1)} \f$ is called the \e base point.
         *
         * This class is used to represent points on rays for a plane-probing estimator, so in practice the point \f$ q \f$ is the fixed point
         * and the three vectors \f$ (m_k)_{0 \leq k \leq 2} \f$ are the vectors defining the current probing frame.
         *
         * @tparam Integer the integer type, model of concepts::CInteger.
         */
      	template < concepts::CInteger Integer = int, typename Index = std::size_t >
        class PointOnProbingRay
        {
            // ----------------------- Public types ------------------------------
            public:
                using Permutation = std::array<Index, 3>;

            public:
                /**
                 * Default constructor.
                 */
                PointOnProbingRay () = default;

                /**
                 * Constructs a ray with a permutation and an index.
                 *
                 * @param aSigma a permutation.
                 * @param aInt an integer that determines a point along the ray.
                 */
                PointOnProbingRay (Permutation const& aSigma, Integer const& aInt = Integer(0));

                /**
                 * @return the base point of the ray (with index 0).
                 */
                PointOnProbingRay getBase () const;

                /**
                 * @return the permutation that defines the ray.
                 */
                Permutation const& sigma () const;

                /**
                 * @param aIndex an index between 0 and 2.
                 * @return the i-th element of the permutation that defines the ray.
                 */
	        Index sigma (Index const& aIndex) const;

                /**
                 * @return integer that locates the current point on the ray.
                 */
                Integer const& position () const;

	        /**
		 * Returns the geometric realization of this grid point. 
		 *
		 * @param aM an array of three points. 
		 * @tparam Point a type for points. 
		 * @return the computed point. 
		 */
	         template < typename Point >
		 Point relativePoint (std::array<Point, 3> const& aM) const;
	  
                /**
                 * Equality test between two rays: the internal permutations and
                 * indices must be the same.
                 *
                 * @param aRay an other ray.
                 * @return true if the two rays are the same, false otherwise.
                 */
                bool operator== (PointOnProbingRay const& aRay) const;

                /**
                 * Inequality test between two rays.
                 *
                 * @param aRay an other ray.
                 * @return true if the two rays are different, false otherwise.
                 */
                bool operator!= (PointOnProbingRay const& aRay) const;

                /**
                 * Comparison operator between two rays: one ray is less than another if they have
                 * the same internal permutation and the first one has a smaller index than the second
                 * one.
                 *
                 * @param aRay an other ray.
                 * @return true if *this <= aRay, false otherwise.
                 *
                 */
                bool operator<= (PointOnProbingRay const& aRay) const;

                /**
                 * @param aInc an increment.
                 * @return a new point on a ray, with index the current index incremented by aInc.
                 */
                PointOnProbingRay next (Integer const& aInc) const;

                /**
                 * @param aDec a decrement.
                 * @return a new point on a ray, with index the current index decremented by aInc.
                 */
                PointOnProbingRay previous (Integer const& aDec) const;

            private:
                Permutation mySigma; /**< The permutation. */
                Integer myPosition; /**< The index. */
        }; // end of class PointOnProbingRay

        /**
         * Display a probing ray on the standard output.
         *
         * @param aOs the output stream where the object is written.
         * @param aRay the probing ray to display.
         * @return the output stream after the writing.
         */
      template < typename Integer, typename Index >
      std::ostream& operator<< (std::ostream& aOs, PointOnProbingRay<Integer,Index> const& aRay);

      /////////////////////////////////////////////////////////////////////////////
      // template class GridPoint
      /**
       * Description of template class 'GridPoint' <p>
       * \brief
       * A grid point consists of a couple of nonnegative coordinates \f$ (x,y)  \f$ and an integer index \f$ k \f$
       * that determines a point used as origin. 
       * For a triplet of vectors \f$ (m_k)_{0 \leq k \leq 2} \f$ and a point \f$ q \f$, a grid point is defined as:
       * \f$ q - m_{k} + x m_{(k+1)\bmod 3} + y m_{(k+2)\bmod 3} \f$. \f$ q - m_{k} \f$, called base point, is used
       * as origin. 
       *
       * This class is used to represent candidate points for the plane-probing L-algorithm. In practice, 
       * the point \f$ q \f$ is the fixed point and the three vectors \f$ (m_k)_{0 \leq k \leq 2} \f$ are 
       * the vectors defining the current probing frame.
       *
       * @tparam Integer the integer type, model of concepts::CInteger.
       */
      template < concepts::CInteger Integer = int, typename Index = std::size_t >
      class GridPoint
      {
      public:
	
	/**
	 * Default constructor.
	 */
	GridPoint () = default;

	/**
	 * Constructs a grid point from a couple of coordinates and 
	 * the index of the point used as origin. 
	 *
	 * @param aDir a pair of nonnegative integers. 
	 * @param aK an index in {0,1,2}.
	 */
	GridPoint (std::pair<Integer,Integer> const& aDir, Index const& aK ) : myDir(aDir), myK(aK) {}

	/**
	 * Constructs a grid point from a couple of coordinates and 
	 * the index of the point used as origin. 
	 *
	 * @param aX first coordinate.
	 * @param aY second coordinate. 
	 * @param aK an index in {0,1,2}.
	 */
	GridPoint (Integer const& aX, Integer const& aY, Index const& aK ) : myDir(std::make_pair(aX,aY)), myK(aK) {}
	  
	/**
	 * Returns the couple of coordinates, i.e., 
	 * the direction going from the origin to 
	 * the grid point. 
	 *
	 * @return the direction. 
	 */
	std::pair<Integer,Integer> direction() const {
	  return myDir; 
	}

	/**
	 * Returns the index of the point used as origin. 
	 *
	 * @return the index. 
	 */
	Index k() const {
	  return myK; 
	}
	
	/**
	 * Returns the vector going from the base point to the 
	 * geometric realization of this object. 
	 *
	 * @param aM an array of three vectors. 
	 * @tparam Vector a type for vectors. 
	 * @return the computed vector. 
	 */
	template < typename Vector >
	Vector directionVector (std::array<Vector, 3> const& aM) const {
	  return aM[(myK+1)%3]*myDir.first + aM[(myK+2)%3]*myDir.second;
	}

	/**
	 * Returns the geometric realization of this grid point. 
	 *
	 * @param aM an array of three points. 
	 * @tparam Point a type for points. 
	 * @return the computed point. 
	 */
	template < typename Point >
	Point relativePoint (std::array<Point, 3> const& aM) const {
	  return -aM[myK] + directionVector(aM); 
	}
	
	/**
	 * Tells whether this grid point is equal to another or not.
	 * Two grid points are equal iff their members are equal. 
	 *
	 * @param other another grid point. 
	 * @return 'true' if equal, 'false' otherwise.
	 */
	bool operator== (GridPoint const& other) const {
	  return (myDir == other.myDir) && (myK == other.myK);
	}

	/**
	 * Tells whether this grid point is different from another or not.
	 *
	 * @param other another grid point. 
	 * @return 'true' if different, i.e., not equal, 'false' otherwise. 
	 */
	bool operator!= (GridPoint const& other) const {
	  return !(*this == other);
	}

	/**
	 * Returns a grid point given a couple of coordinates.  
	 *
	 * @param aDir a couple of coordinates. 
	 * @return the resulting grid point.  
	 */
	GridPoint getOnSameGrid(const std::pair<Integer,Integer>& aDir) const {   
	  return GridPoint(aDir,myK);                                    
	}

	/**
	 * Adds a grid point to this one (as if they were vectors). 
	 *
	 * @param other another grid point. 
	 * @return the resulting point.  
	 */
	GridPoint operator+(const GridPoint & other) const {   
	  ASSERT(myK == other.myK);
	  std::pair<Integer,Integer> d = std::make_pair(myDir.first+other.myDir.first,
						myDir.second+other.myDir.second);
	  return getOnSameGrid(d);                                    
	}

	/**
	 * Scales this grid point by a scalar (as if it was vector). 
	 *
	 * @param aValue a scalar value
	 * @return the resulting point.  
	 */
	GridPoint operator*(Integer aValue) const {   
	  std::pair<Integer,Integer> d = std::make_pair(myDir.first*aValue,
						myDir.second*aValue);
	  return getOnSameGrid(d);                                    
	} 
	
	/**
	 * Checks whether the representation of the grid point is valid, 
	 * i.e., the coordinates are nonnegative coordinates, not both 
	 * equal to zero, and the index is in {0,1,2}. 
	 *
	 * @return 'true' if valid, 'false' otherwise. 
	 */
	bool isValid() const {
	  if ( (myDir.first != 0) || (myDir.second != 0) ) { //not both null
	    return ( (myDir.first >= 0) && (myDir.second >= 0)
		     && (myK >= 0) && (myK <= 2)
		     );
	  } else {
	    return false; 
	  }
	}
	
      private:

	std::pair<Integer,Integer> myDir; /**< Couple of coordinates giving a direction */
	Index myK; /**< Index of a point used as origin */	
	
      }; //end of class GridPoint

      /**
       * Display a grid point on the standard output.
       *
       * @param aOs the output stream where the object is written.
       * @param aGridPoint the grid point to display.
       * @return the output stream after the writing.
       */
      template < typename Integer, typename Index >
      std::ostream& operator<< (std::ostream& aOs, GridPoint<Integer,Index> const& aGridPoint) {
	aOs << "GridPoint[k=" << aGridPoint.k()
	    << ", a=" << aGridPoint.direction().first
	    << ", b=" << aGridPoint.direction().second
	    << "]";
	return aOs; 
      }

      /////////////////////////////////////////////////////////////////////////////
      // template class GridPointOnProbingRay
      /**
       * Description of template class 'GridPointOnProbingRay' <p>
       * \brief Aim: Represents a grid point along a discrete ray defined on a grid. 
       *
       * More precisely, a ray consists of a starting point (represented as an instance of 'GridPoint')
       * and a direction (respresented as a couple of coordinates for the basis of the underlying grid). 
       * A grid point along that ray is determined by an index, 0 being the starting point of the ray. 
       * The class provides several methods to compare and move points along the ray.  
       *
       * @tparam Integer the integer type, model of concepts::CInteger.
       */
      template < concepts::CInteger Integer = int, typename Index = std::size_t >
      class GridPointOnProbingRay
      {
      public: 
	/**
	 * Default constructor.
	 */
	GridPointOnProbingRay () = default;

	/** 
	 * Constructor.
	 *
	 * @param aGridPoint starting point of the ray
	 * @param aDirection direction of the ray
	 * @param aIdx index of the grid point along the ray
	 */
	GridPointOnProbingRay (const GridPoint<Integer, Index>& aGridPoint,
			       const std::pair<Integer,Integer>& aDirection,
			       const Integer& aIdx = 0)
	  : myOrigin(aGridPoint), myDirection(aDirection), myIdx(aIdx) {}

	/**
	 * Equality test. The two objects are equal iff 
	 * the underlying rays are the same and 
	 * the indices are the same. 
	 *
	 * @param other another instance of GridPointOnProbingRay
	 * @return 'true' if equal, 'false' otherwise
	 */
	bool operator== (GridPointOnProbingRay const& other) const {
	  return ( (myOrigin == other.myOrigin) &&
		   (myDirection == other.myDirection) &&
		   (myIdx == other.myIdx) );
	}

	/**
	 * Difference test.   
	 *
	 * @param other another instance of GridPointOnProbingRay
	 * @return 'true' if different, i.e. not equal, 'false' otherwise
	 */
	bool operator!= (GridPointOnProbingRay const& other) const {
	  return !(*this == other);
	}

	/**
	 * Returns a grid point lying after this one along the ray. 
	 * The distance is given as an input parameter. 
	 *
	 * @param aInc an increment.
	 * @return a new grid point on the same ray, with index,
	 * the current index incremented by 'aInc'.
	 */
	GridPointOnProbingRay next(const Integer& aInc) const {
	  return GridPointOnProbingRay(myOrigin, myDirection, myIdx+aInc); 
	}

	/**
	 * Returns a grid point lying before this one along the ray. 
	 * The distance is given as an input parameter. 
	 *
	 * @param aDec a decrement.
	 * @return a new grid point on the same ray, with index,
	 * the current index decremented by 'aDec'.
	 */
	GridPointOnProbingRay previous(const Integer& aDec) const {
	  return GridPointOnProbingRay(myOrigin, myDirection, myIdx-aDec); 
	}
	
	/**
	 * @return index of the current grid point on the ray.
	 */
	Integer index() const {
	  return myIdx; 
	}

	/**
	 * @return the current grid point as an instance of GridPoint.
	 */
	GridPoint<Integer, Index> gridPoint() const {
	  return myOrigin + myOrigin.getOnSameGrid(myDirection)*myIdx; 
	}

	/**
	 * Returns the geometric realization of this grid point. 
	 *
	 * @param aM an array of three points. 
	 * @tparam Point a type for points. 
	 * @return the computed point. 
	 */
	template < typename Point >
	Point relativePoint (std::array<Point, 3> const& aM) const {
	  return gridPoint().relativePoint(aM); 
	}


      private: 
	GridPoint<Integer, Index> myOrigin; /**< starting point of the ray */
	std::pair<Integer, Integer> myDirection; /**< direction of the ray */
	Integer myIdx; /**< index of the point along the ray */
	
      }; //end of class GridPointOnProbingRay
      
    } // namespace detail
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/helpers/PlaneProbingEstimatorHelper.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingEstimatorHelper_h

#undef PlaneProbingEstimatorHelper_RECURSES
#endif // else defined(PlaneProbingEstimatorHelper_RECURSES)
