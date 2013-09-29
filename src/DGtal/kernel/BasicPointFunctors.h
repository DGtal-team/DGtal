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
 * @file BasicPointFunctors.h
 *
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/02/02
 *
 * This files contains several basic classes representing Functors
 * on points.
 *
 * This file is part of the DGtal library.
 */

#if defined(BasicPointFunctors_RECURSES)
#error Recursive header files inclusion detected in BasicPointFunctors.h
#else // defined(BasicPointFunctors_RECURSES)
/** Prevents recursive inclusion of headers. */
#define BasicPointFunctors_RECURSES

#if !defined BasicPointFunctors_h
/** Prevents repeated inclusion of headers. */
#define BasicPointFunctors_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <iterator>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/base/BasicBoolFunctions.h"
#ifdef CPP11_ARRAY
#include <array>
#else
#include <boost/array.hpp>
#endif
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class Projector
  /**
   * Description of template class 'Projector' <p>
   * \brief Aim: Functor that maps 
   * a point P of dimension i
   * to a point Q of dimension j.
   * The member @a myDims is an array containing the 
   * coordinates - (0, 1, ..., j-1) by default -
   * that are copied from P to Q. 
   *
   * Ex: for i = 3 and j = 2, the first two coordinates 
   * (numbered 0 and 1) are copied so that point (x,y,z) is 
   * is mapped to point (x,y).    
   * 
   * All kth coordinates (0 < k < j) that are greater than i, 
   * are set to a value given at construction (0 by defaut). 
   *
   * Ex: for i = 2 and j = 3, the first two coordinates 
   * (numbered 0 and 1) are copied so that point (x,y) is 
   * is mapped to point (x,y,0).    
   *
   * Instead of using the default order, you can define your own 
   * orthonormal basis as shown below: 
   *
   * @code

   PointVector<3,int> p(3,1,5): 

   Projector<SpaceND<2,int> > proj; 
   proj( p ) //== (3,1)

   ...
   //v (2, 0): selection of the 2nd and 0th basis vectors
   proj.init(v.begin(), v.end()); 
   proj( p ) //== (5,3)

   * @endcode
   *
   * @tparam S type for the space where must lie the projected point
   */



   



  template <typename S = SpaceND< 2, DGtal::int32_t > >
  struct Projector
  {

    
    typedef S Space; 
    typedef typename Space::Dimension Dimension;
    static const Dimension dimension;
    typedef typename Space::Integer Integer; 
    typedef typename Space::Point Point; 




    /**
     * Default constructor
     */
    Projector(const Integer& aDefaultInteger = NumberTraits<Integer>::zero());

    
    /**
     * Initialization of the array of relevant dimensions 
     * @param itb begin iterator on dimensions.
     * @param ite end iterator on dimensions.
     */
    template<typename TIterator>
    void init ( const TIterator& itb, const TIterator& ite );


    /**
     *  Initialisation by removing a given dimension.
     *  @param dimRemoved the removed dimension.
     */
    void initRemoveOneDim ( const Dimension  &dimRemoved );

    /**
     *  Initialisation by adding a given dimension.
     *  @param newDim the new dimension.
     */
    void initAddOneDim ( const Dimension & newDim );


    /**
     *  Main operator
     * @param aPoint any point.
     * @return the projected point.
     */
    template<typename TInputPoint>
    Point operator()( const TInputPoint& aPoint ) const;

  private: 
    /**
     * Array storing the coordinates that are copied from 
     * the input point to its projection (order matters)
     */
#ifdef CPP11_ARRAY
    std::array<Dimension, Space::dimension> myDims; 
#else
    boost::array<Dimension, Space::dimension> myDims; 
#endif
    /**
     * Default integer set to coordinates of the projected point
     * not in the input point
     */
    Integer myDefaultInteger; 

  }; // end of class ConstantPointFunctors
  




  /**
   * Description of template class 'SliceRotator2D' <p>
   * \brief Special Point Functor that adds one dimension to a 2D point and
   *  apply on it a rotation of angle alpha according to a given
   *  direction and the domain center. It also checks if the resulting
   *  point is inside the 3D domain, else it returns a particular point (by
   *  default the point at domain origin lowBound() point).
   *
   * Ex: a Point P (10, 9) in the domain (defined (0,0,0) (10,10,10))
   * given in 3D by adding the dimension in Z (2) with slice num 7: =>
   * P(10, 9, 7) and after a rotation of PI from center of slice domain (5,5, 7)
   * will give P(0,1,7). 
   * To apply this example you can test it with:
   * @code 
   // Defining the domain
   PointVector<3, int> pt1(0,0, 0);
   PointVector<3, int> pt2(10,10, 10);
   HyperRectDomain<SpaceND<3, int> >  domain (pt1, pt2);
   // The functor on axis rotation set to 2 with new Z slice num=7 and angle 3.15: 
   SliceRotator2D< HyperRectDomain<SpaceND<3, int> >, int> sliceRot2(2, domain, 7, 2, 3.14);
   PointVector<2, int> pt_2(10, 9);  
   trace.info() <<  sliceRot2(pt_2);
   @endcode
   *
   *
   * @tparam TDomain3D the type of the 3d domain. 
   * @tparam TInteger specifies the integer number type used to define the space. 
   *
   */
  template <typename TDomain3D, typename TInteger =  DGtal::int32_t >
  class SliceRotator2D
  {
  public:
        
    typedef SpaceND< 3, TInteger>  Space;
    typedef typename Space::Dimension Dimension; 
    typedef typename Space::Point Point; 
    typedef typename Space::Integer Integer; 
    
    /** 
     * Constructor.
     * @param dimAdded  the index of the new dimension inserted.
     * @param aDomain3DImg the 3D domain used to keep the resulting point in the domain. 
     * @param sliceIndex the value that is used to fill the dimension for a given N-1 point (equivalently the slice index).  
     * @param dimRotated the index of the rotation axis.
     * @param rotationAngle the angle of rotation (in radians).
     * @param defautPoint the point given when the resulting point is outside the domain (default Point(0,0,0)).
     */
    
    SliceRotator2D( const Dimension &dimAdded, const TDomain3D &aDomain3DImg, 
		    const Integer &sliceIndex,  const Dimension &dimRotated,
		    double rotationAngle, const Point &defautPoint = Point(0,0,0)):
      myPosDimAdded(dimAdded), mySliceIndex(sliceIndex), myDomain(aDomain3DImg), 
      myDimRotated(dimRotated), myRotationAngle(rotationAngle), myDefaultPoint (defautPoint)
    {
      myCenter[0] = ((aDomain3DImg.upperBound())[0]-(aDomain3DImg.lowerBound())[0])/2.0;  
      myCenter[1] = ((aDomain3DImg.upperBound())[1]-(aDomain3DImg.lowerBound())[1])/2.0;  
      myCenter[2] = ((aDomain3DImg.upperBound())[2]-(aDomain3DImg.lowerBound())[2])/2.0;  
      myCenter[dimAdded]=sliceIndex;
    };
    /** 
     * Constructor.
     * @param dimAdded  the index of the new dimension inserted.
     * @param aDomain3DImg the 3D domain used to keep the resulting point in the domain. 
     * @param sliceIndex the value that is used to fill the dimension for a given N-1 point (equivalently the slice index).  
     * @param dimRotated the index of the rotation axis.
     * @param ptCenter the rotation center.
     * @param rotationAngle the angle of rotation (in radians).
     * @param defautPoint the point given when the resulting point is outside the domain (default Point(0,0,0)).
     */
    
    SliceRotator2D( const Dimension &dimAdded, const TDomain3D &aDomain3DImg, const Integer &sliceIndex,
		    const Dimension &dimRotated,  const Point &ptCenter, double rotationAngle, const Point &defautPoint = Point(0,0,0)):
      myPosDimAdded(dimAdded), mySliceIndex(sliceIndex), myDomain(aDomain3DImg), 
      myDimRotated(dimRotated), myRotationAngle(rotationAngle), myCenter(ptCenter), myDefaultPoint (defautPoint)
    {
      myDefaultPoint = Point(0,0,0);
    };
    
    /** 
     * The operator just recover the 3D Point associated to the SliceRotator2D parameters.
     * @param[in] aPoint point of the input domain (of dimension N-1).
     * 
     * @return the point of dimension 3.
     */
    template <typename TPointDimMinus>
    inline
    Point  operator()(const TPointDimMinus& aPoint) const
    {
      Point pt;
      Dimension pos=0;
      std::vector<Dimension> indexesRotate;
      for( Dimension i=0; i<pt.size(); i++)
	{
	  if(i!=myPosDimAdded)
	    {
	      pt[i]= aPoint[pos];
	      pos++; 
	    }else
	    {
	      pt[i]=mySliceIndex;
	    }
	}
      for( Dimension i=0; i<pt.size(); i++)
	{
	  if(i!=myDimRotated)
	    indexesRotate.push_back(i);
	}
      double d1 = pt[indexesRotate[0]] - myCenter[indexesRotate[0]];
      double d2 = pt[indexesRotate[1]] - myCenter[indexesRotate[1]];
      
      pt[indexesRotate[0]] = myCenter[indexesRotate[0]] + d1*cos(myRotationAngle)-d2*sin(myRotationAngle) ; 
      pt[indexesRotate[1]] = myCenter[indexesRotate[1]] + d1*sin(myRotationAngle)+d2*cos(myRotationAngle) ; 
      
      if(myDomain.isInside(pt))
	return pt;
      else
	return  myDefaultPoint;
    }
  private:
    // position of insertion of the new dimension
    Dimension myPosDimAdded;
    // the index of the slice associated to the new domain.
    Integer mySliceIndex;
    TDomain3D myDomain;
    PointVector<3, double> myCenter;
    double myRotationAngle;
    Dimension myDimRotated;
    Point myDefaultPoint;
  };



} // namespace dgtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/BasicPointFunctors.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined BasicPointFunctors_h

#undef BasicPointFunctors_RECURSES
#endif // else defined(BasicPointFunctors_RECURSES)
