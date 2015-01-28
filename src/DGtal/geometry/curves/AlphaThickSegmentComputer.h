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
 * @file AlphaThickSegmentComputer.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 * @author Alexandre Faure
 * @author Fabien Feschet 
 * @author Mohammad Said
 * @author Jacques-Olivier Lachaud
 *
 * @date 2015/01/05
 *
 * Header file for module AlphaThickSegmentComputer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(AlphaThickSegmentComputer_RECURSES)
#error Recursive header files inclusion detected in AlphaThickSegmentComputer.h
#else // defined(AlphaThickSegmentComputer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AlphaThickSegmentComputer_RECURSES

#if !defined AlphaThickSegmentComputer_h
/** Prevents repeated inclusion of headers. */
#define AlphaThickSegmentComputer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/ParallelStrip.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class AlphaThickSegmentComputer
/**
 * Description of class 'AlphaThickSegmentComputer' <p>
 * \brief Aim:
 */


template <typename TSpace, 
          typename TInputPoint,
          typename TInternalScalar>

class AlphaThickSegmentComputer
{

  // ----------------------- public types --------------------------------------
  BOOST_CONCEPT_ASSERT(( concepts::CSpace< TSpace > ));
  BOOST_CONCEPT_ASSERT(( concepts::CSignedNumber< TInternalScalar > ));
  BOOST_STATIC_ASSERT(( TSpace::dimension == 2 ));
  BOOST_STATIC_ASSERT(( TInputPoint::dimension == 2 ));
  
public:
  typedef TSpace Space;
  typedef TInputPoint InputPoint;
  typedef TInternalScalar InternalScalar;
  typedef InputPoint InputVector;
  typedef InternalScalar InternalVector[ 2 ];

  
  typedef std::vector< InputPoint > InputPointContainer;
  typedef typename InputPointContainer::size_type Size;
  typedef typename InputPointContainer::const_iterator ConstIterator;
  typedef typename InputPointContainer::iterator Iterator;
  typedef ParallelStrip<Space,true,true> Primitive;  


  // ----------------------- internal types --------------------------------------

private: 
  struct State{
    std::deque<unsigned int > melkmanQueue; /** Melkman algorithm main dequeu */
    InputPoint lastFront; /** the last point added at the front of the alpha thick segment */
    InputPoint lastBack; /** the last point added at the back of the alpha thick segment */
    InputPoint edgePh; /** one the convexhull edge point of the (edge, vertex) pair used to compute the convexhull height */
    InputPoint edgeQh; /** one the convexhull edge point of the (edge, vertex) pair used to compute the convexhull height */
    InputPoint vertexSh; /** one the convexhull vertex of the (edge, vertex) pair used to compute the convexhull height */
    bool isMelkmanInitialized; /** well initialized when at least 3 points are given.  */ 
    double convexHullHeight;  /** Used in melkmanMainDiagonal() */
    double convexHullWidth;  /** Used in melkmanMainDiagonal() */ 
  };
    
  
  // ----------------------- Standard services ------------------------------
public:
  

  /**
   * Constructor.
   */
  AlphaThickSegmentComputer();


  /**
   * Destructor.
   */
  ~AlphaThickSegmentComputer();

  /**
   * Initialisation.
   * @param[in] aThickness the maximal thickness of the alpha thick segment (alpha_max).
   */  
  void init(double aThickness);  
  
  


  //-------------------- model of ForwardContainer -----------------------------
public:

  /**
   * @return the number of distinct points in the current alpha thick segment.
   */
  Size size() const;
  
  /**
   * @return 'true' if and only if this object contains no point.
   */
  bool empty() const;
  
  
  /**
   * @return a const iterator pointing on the first point stored in the current alpha thick segment.
   */
  ConstIterator begin() const;


  /**
   * @return a const iterator pointing after the last point stored in the current alpha thick segment.
   */
  ConstIterator end() const;


  /**
   * NB: std version.
   * @return the maximal allowed number of points in the current alpha thick segment.
   * @see maxSize
   */
  Size max_size() const;

  
  /**
   * same as max_size
   * @return the maximal allowed number of points in the current alpha thick segment.
   */
  Size maxSize() const;  
  



  //-------------------- model of CIncrementalPrimitiveComputer ----------------

public:
     
  /**
   *  Tests whether the current alpha thick segment can be extended, i.e
   *  checks if we have still an alpha thick segment of width alpha_max after
   *  adding the given point \aPoint. The segment parameters are keep
   *  in its original state.
   *  
   * @param[in] aPoint the point to be tested for the segment extension.
   * @return 'true' if the segment can be extended with the given point, 'false' otherwise.
   */
  bool isExtendable(const InputPoint &aPoint);


  /**
   * Tries to add the point \a aPoint at the front of the current alpha thick
   * segment and checks if we have still an alpha thick segment of thickness
   * less or equal to the initial value alpha_max. If it is the case
   * the new point is added and the segment parameters are updated,
   * otherwise the alpha thick segment is keep in its original state.
   *
   * @param[in] aPoint the new point to extend the current segment.
   * @return 'true' if the segment has been extended and 'false'
   * otherwise (the object is then in its original state).
   */
  bool extend(const InputPoint &aPoint);
  
 

  //-------------------- Primitive services -----------------------------
  
  
  /**
     Returns the current primitive recognized by this computer,
     which is a ParallelStrip of axis width smaller than the one
     specified at instanciation.
  */
  Primitive primitive() const;
  


  
    // ----------------------- Interface --------------------------------------
public:

  
  /**
   * Checks the validity/consistency of the object.
   * @return 'true' if the object is valid, 'false' otherwise.
   */
  bool isValid() const;
  


 
   /**
   * Returns the segment length defined from the basic bouding box (@see getBasicBoundingBox).
   *
   * @return the segment length.
   **/
  double getBasicLength();


  /**
   * Returns the segment length defined from the real bouding box (@see getRealBoudingBox).
   *
   * @return the segment length.
   **/
  double getRealLength();
  


  // ------------------------- Display services ------------------------------


  /**
   * Returns the current segment convexhull.
   * 
   * @return the convexhull given as a vector containing the vertex points.
   **/
  std::vector<InputPoint> getConvexHull() const;


  
  /**
   * Computes the segment bounding box according to two extremity points (\a aFirstPt, \a aLastPt).
   * @param[in] aFirstPt the first extrem point.
   * @param[in] aLastPt the last extrem point. 
   * @param[out] pt1LongestSegment1 the first point of one of the longest segment.
   * @param[out] pt2LongestSegment1 the second point of one of the longest segment.
   * @param[out] pt3LongestSegment2 the first point of one of the second longest segment.
   * @param[out] pt4LongestSegment2 the second point of one of the second longest segment.
   * @param[in] minVisibleWidthBounds the minimal width of the resulting bounding box (for drawing issue).
   *
   * @note the segment bounding box can be drawn with the sequence of
   * out parameters pt1LongestSegment1, pt2LongestSegment1,
   * pt3LongestSegment1, pt4LongestSegment1.
   **/
  
  void getBoundingBoxFromExtremPoints(const InputPoint &aFirstPt,
                                      const InputPoint &aLastPt,
                                      InputPoint &pt1LongestSegment1,
                                      InputPoint &pt2LongestSegment1,
                                      InputPoint &pt3LongestSegment2,
                                      InputPoint &pt4LongestSegment2,
                                      double minVisibleWidthBounds = 0.2) const;
  
  
  /**
   * Computes the basic segment bounding box according to the segment
   * extremity points (the last points added to the front and to the
   * back).
   *
   * @param[out] pt1LongestSegment1 the first point of one of the longest segment.
   * @param[out] pt2LongestSegment1 the second point of one of the longest segment.
   * @param[out] pt3LongestSegment2 the first point of one of the second longest segment.
   * @param[out] pt4LongestSegment2 the second point of one of the second longest segment.
   * @param[in] minVisibleWidthBounds the minimal width of the resulting bounding box (for drawing issue).
   *
   * @note the segment bounding box can be drawn with the sequence of
   * out parameters pt1LongestSegment1, pt2LongestSegment1,
   * pt3LongestSegment1, pt4LongestSegment1.
   **/

  void getBasicBoundingBox(InputPoint &pt1LongestSegment1,
                           InputPoint &pt2LongestSegment1,
                           InputPoint &pt3LongestSegment2,
                           InputPoint &pt4LongestSegment2,
                           double minVisibleWidthBounds=0.2) const;
  
  
  /**
   * Computes the real bounding box according to the real extremity
   * points of the segment. The real extremity points are computed
   * after a scan of all the segment points. The real bounding box can
   * differs of the basic bounding box when a large amount of noise
   * are give in the initial curve.
   *
   * @param[out] pt1LongestSegment1 the first point of one of the longest segment.
   * @param[out] pt2LongestSegment1 the second point of one of the longest segment.
   * @param[out] pt3LongestSegment2 the first point of one of the second longest segment.
   * @param[out] pt4LongestSegment2 the second point of one of the second longest segment.
   * @param[in] minVisibleWidthBounds the minimal width of the resulting bounding box (for drawing issue).
   *
   * @note the segment bounding box can be drawn with the sequence of
   * out parameters pt1LongestSegment1, pt2LongestSegment1,
   * pt3LongestSegment1, pt4LongestSegment1.
   **/
  void  getRealBoundingBox(InputPoint &pt1LongestSegment1,
                          InputPoint &pt2LongestSegment1,
                          InputPoint &pt3LongestSegment2,
                          InputPoint &pt4LongestSegment2,
                          double minVisibleWidthBounds=0.2) const;
  
  
  

   /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;



  
  /**
   * Writes/Displays the object on an output stream.
   *
   * @param[out] out the output stream where the object is written.
   */
  void selfDisplay ( std::ostream & out ) const;
  
    // ------------------------- Protected Datas ------------------------------
protected:
  
  
  
  /**
   * begin iterator (associated to input data)
   **/
  ConstIterator myBegin;
  
  /**
   * begin iterator (associated to input data)
   **/
  ConstIterator myEnd;
  
    


    // ------------------------- Private Datas --------------------------------
private:

  /**
   * The set of points contained in the alpha thick segment which can be changed during computations.
   **/
  mutable InputPointContainer myPointContainer; 

  /**
   * The maximal thickness of the segment.
   */
  double myThickness;  
  
  State myState;
  
  mutable State _state;
  
  
  


    // ------------------------- Hidden services ------------------------------
protected:


  /**
   * Copy constructor.
   *
   * @param[in] other the object to clone.
   * Forbidden by default.
   */
  AlphaThickSegmentComputer ( const AlphaThickSegmentComputer & other );
  
  /**
   * Assignment.
   *
   * @param other the object to copy.
   * @return a reference on 'this'.
   * Forbidden by default.
   */
  AlphaThickSegmentComputer & operator= ( const AlphaThickSegmentComputer & other );
  
  
  /**
   *  Used to check if the initialisation with 3 non aligned points is well done.
   *
   * @return 'true' if the segment contains at least 3 non aligned points. 
   **/     
   bool melkmanIsWellInitialized() const;


  /**
   *  Updates the main height of the melkman convex set.
   **/     
  void melkmanUpdateMainHeight();
  
  
  /**
   * Adds a point in the convex set  using one step of Melkman algorithm.
   *
   * @param[in] aPointIndex the point to be added.
   */
  void melkmanAddPoint( unsigned int aPointIndex );



  /**
   * Checks if the 3 init points of the queue are given in the correct order (else it re order it).
   * (it checks only the 3 first points of \a myState.melkmanQueue).
   **/
  void melkmanInitCheck( ) ; 


  /**
   * Depending on connexity, return true if a convex is valid.
   */
  bool melkmanIsConvexValid() ;
  
  
  /**
   * Tests if the point of index \a aPointIndex2 is Left|On|Right of an infinite line (defined from two points of index aPointIndex0 and aPointIndex1).
   *
   * @param[in] aPointIndex0 the first point defining the line.
   * @param[in] aPointIndex1 the second point defining the line.
   * @param[in] aPointIndex2 the point to be tested.
   */
  InternalScalar melkmanIsLeft(unsigned int aPointIndex0, unsigned int aPointIndex1, unsigned int aPointIndex2) const;
  
  
  /**
   * Compute the projection of a Point \a ptC on the real line defined by the two points (\a ptA, \a ptB), and
   * return true if the projected point is inside the segment closed interval [A,B].
   * 
   * @param[in] ptA one of the two points defining the straight line.
   * @param[in] ptB one of the two points defining the straight line.
   * @param[in] ptC the point to be projected.
   * @param[out] ptProjected (return) the projected point.
   * @return true if ptProjected is inside the segment [A,B].
   **/
  
  bool projetOnStraightLine(const InputPoint & ptA, const InputPoint & ptB,
                            const InputPoint & ptC,
                            InputPoint & ptProjected) const;
  

    // ------------------------- Internals ------------------------------------
private:

}; // end of class AlphaThickSegmentComputer


/**
 * Overloads 'operator<<' for displaying objects of class 'AlphaThickSegmentComputer'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'AlphaThickSegmentComputer' to write.
 * @return the output stream after the writing.
 */
template <typename TSpace, typename TInputPoint, typename TInternalScalar>
std::ostream&
operator<< ( std::ostream & out, const AlphaThickSegmentComputer<TSpace, TInputPoint, TInternalScalar> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/geometry/curves/AlphaThickSegmentComputer.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined AlphaThickSegmentComputer_h

#undef AlphaThickSegmentComputer_RECURSES
#endif // else defined(AlphaThickSegmentComputer_RECURSES)
