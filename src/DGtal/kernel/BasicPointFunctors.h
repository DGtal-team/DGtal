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
   * Create a special Point Functor that add one dimension to a 2D
   *  point and apply a rotation of angle alpha to the resulting 3D
   *  point according a given direction and a center.
      *
   * @tparam TSpace type for the space where must lie the projected point
   *
   */
  template <typename TDomain, typename TInteger =  DGtal::int32_t >
  class SliceRotator2D
  {
  public:
        
    typedef SpaceND< 3, TInteger>  Space;
    typedef typename Space::Dimension Dimension; 
    typedef typename Space::Point Point; 
    typedef typename Space::Integer Integer; 
    
    /** 
     * Constructor.
     * @param  posDimAdded  position of insertion of the new dimension.
     * @param sliceIndex the value that is used to fill the dimension for a given N-1 point (equivalently the slice index).  
     */
    // dimRotated dim of the axis of rotation i.e point[dimRotated] = cst;
    
    SliceRotator2D( Dimension dimAdded, TDomain aDomain3DImg, Integer sliceIndex,  Dimension dimRotated, double angle, Point center):
      myPosDimAdded(dimAdded), mySliceIndex(sliceIndex), myDomain(aDomain3DImg), 
      myDimRotated(dimRotated), myAngle(angle), myCenter(center) {};
    
    /** 
     * The operator just recover the ND Point associated to the slice parameter.
     * @param[in] aPoint point of the input domain (of dimension N-1).
     * 
     * @return the point of dimension N.
     */
    template <typename TPointDimMinus>
    inline
    Point  operator()(const TPointDimMinus& aPoint) const
    {
      Point pt;
      Dimension pos=0;
      std::vector<Dimension> indexesRotate;
      for( Dimension i=0; i<pt.size(); i++){
	if(i!=myPosDimAdded){
	  pt[i]= aPoint[pos];
	  pos++; 
	}else{
	  pt[i]=mySliceIndex;
	}
      }
      for( Dimension i=0; i<pt.size(); i++){
	if(i!=myDimRotated)
	  indexesRotate.push_back(i);
      }
      double d1 = pt[indexesRotate[0]] - myCenter[indexesRotate[0]];
      double d2 = pt[indexesRotate[1]] - myCenter[indexesRotate[1]];
      
      pt[indexesRotate[0]] = myCenter[indexesRotate[0]] + d1*cos(myAngle)-d2*sin(myAngle) ; 
      pt[indexesRotate[1]] = myCenter[indexesRotate[1]] + d1*sin(myAngle)+d2*cos(myAngle) ; 
      
      if(myDomain.isInside(pt))
	return pt;
      else
	return  Point(0,0,0);
    }
  private:
    // position of insertion of the new dimension
     Dimension myPosDimAdded;
    // the index of the slice associated to the new domain.
    Integer mySliceIndex;
    TDomain myDomain;
    Point myCenter;
    double myAngle;
    Dimension myDimRotated;
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
