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
 * @file FMMPointFunctors.h
 *
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr ) 
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 * @date 2012/02/21
 *
 * @brief Distance computation within a small neighborhood around a point
 *
 * This file is part of the DGtal library.
 *
 */

#if defined(FMMPointFunctors_RECURSES)
#error Recursive header files inclusion detected in FMMPointFunctors.h
#else // defined(FMMPointFunctors_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FMMPointFunctors_RECURSES

#if !defined FMMPointFunctors_h
/** Prevents repeated inclusion of headers. */
#define FMMPointFunctors_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <limits>
#include <vector>
#include <queue>
#include "DGtal/base/Common.h"

#include "DGtal/images/CImage.h"
#include "DGtal/images/ImageHelper.h"
#include "DGtal/kernel/sets/CDigitalSet.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class L2FirstOrderLocalDistance
  /**
   * Description of template class 'L2FirstOrderLocalDistance' <p>
   * \brief Aim: Class for the computation of the Euclidean distance
   * at some point p, from the available distance values of some points 
   * lying in the 1-neighborhood of p (ie. points at a L1-distance to p
   * equal to 1). 
   *
   * The computed value is such that the upwind gradient of the 
   * distance map is one, ie. it the minimum solution \f$ \Phi \f$ 
   * over all quadrants, verifying the following quadratic equation:
   * \f$ \sum_{i = 1 \ldots d } ( \Phi - \Phi_i )^2 \f$
   * where \f$ \Phi_i \f$ is the distance value of the point preceding
   * or following p along the \f$ i \f$ axis. 
   *
   * It is a model of CPointFunctor.
   *
   * @tparam TImage model of CImage used for the mapping point-distance value
   * @tparam TSet model of CDigitalSet for storing points whose distance value is known
   */
  template <typename TImage, typename TSet>
  class L2FirstOrderLocalDistance
  {

    // ----------------------- Types ------------------------------
  public:


    /// image
    BOOST_CONCEPT_ASSERT(( CImage<TImage> ));
    typedef TImage Image;
    typedef typename Image::Point Point;
    typedef typename Image::Value Value; 

    /// set
    BOOST_CONCEPT_ASSERT(( CDigitalSet<TSet> ));
    typedef TSet Set;
    BOOST_STATIC_ASSERT(( boost::is_same< Point, typename TSet::Point >::value ));

  private: 

    typedef std::vector<Value> Values; 
  
    // ----------------------- Data -------------------------------------
  public: 
    /// Aliasing pointer on the underlying image
    Image* myImgPtr; 
    /// Aliasing pointer on the underlying set
    Set* mySetPtr; 


    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Constructor from an image and a set. 
     * NB: only pointers are stored
     *
     * @param aImg any distance map
     * @param aSet any digital set
     */
    L2FirstOrderLocalDistance(Image& aImg, TSet& aSet);

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    L2FirstOrderLocalDistance ( const L2FirstOrderLocalDistance & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    L2FirstOrderLocalDistance & operator= ( const L2FirstOrderLocalDistance & other); 

    /**
     * Destructor.
     * Does nothing.
     */
    ~L2FirstOrderLocalDistance(); 

    /** 
     * Euclidean distance computation at @a aPoint , 
     * from the available distance values
     * of the 1-neighbors of @a aPoint  .
     *
     * @param aPoint the point for which the distance is computed
     *
     * @return the distance value at @a aPoint.
     *
     */
    Value operator() (const Point& aPoint);

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    // ----------------------- Internals -------------------------------------

  private: 

    /**
     * Returns an approximation of the Euclidean distance 
     * at some point, knowing the distance of its neighbors
     * 
     * @param aValueList  the distance of (some of) the neighbors
     * @return the computed distance.
     */
    Value compute(Values& aValueList) const; 


    /**
     * Returns the squared euclidean norm of the gradient 
     * of the distance map
     * 
     * @param aValue  the distance value of the point where the gradient is computed
     * @param aValueList  the distance value of (some of) the neighbors
     *
     * @return the computed gradient norm.
     */
    Value gradientNorm(const Value& aValue, const Values& aValueList) const;
  }; 


  /////////////////////////////////////////////////////////////////////////////
  // template class LInfFirstOrderLocalDistance
  /**
   * Description of template class 'LInfFirstOrderLocalDistance' <p>
   * \brief Aim: Class for the computation of the LInf-distance
   * at some point p, from the available distance values of some points 
   * lying in the 1-neighborhood of p (ie. points at a L1-distance to p
   * equal to 1). 
   *
   * If there is only one available distance value v in the 1-neighborhood of p,
   * the computed value is merely incremented or decremented. 
   * Otherwise, it is the maximum over all
   * the available distance value in the 1-neighborhood of p. 
   *
   * It is a model of CPointFunctor.
   *
   * @tparam TImage model of CImage used for the mapping point-distance value
   * @tparam TSet model of CDigitalSet for storing points whose distance value is known
   */
  template <typename TImage, typename TSet>
  class LInfFirstOrderLocalDistance
  {
    // ----------------------- Types ------------------------------
  public:


    /// image
    BOOST_CONCEPT_ASSERT(( CImage<TImage> ));
    typedef TImage Image;
    typedef typename Image::Point Point;
    typedef typename Image::Value Value; 

    /// set
    BOOST_CONCEPT_ASSERT(( CDigitalSet<TSet> ));
    typedef TSet Set;
    BOOST_STATIC_ASSERT(( boost::is_same< Point, typename TSet::Point >::value ));

  private: 

    typedef std::vector<Value> Values; 

    // ----------------------- Data -------------------------------------
  public: 
    /// Aliasing pointer on the underlying image
    Image* myImgPtr; 
    /// Aliasing pointer on the underlying set
    Set* mySetPtr; 


    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Constructor from an image and a set. 
     * NB: only pointers are stored
     *
     * @param aImg any distance map
     * @param aSet any digital set
     */
    LInfFirstOrderLocalDistance(Image& aImg, TSet& aSet);

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    LInfFirstOrderLocalDistance ( const LInfFirstOrderLocalDistance & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    LInfFirstOrderLocalDistance & operator= ( const LInfFirstOrderLocalDistance & other); 

    /**
     * Destructor.
     * Does nothing.
     */
    ~LInfFirstOrderLocalDistance(); 


    /** 
     * LInf-distance computation at @a aPoint , 
     * from the available distance values
     * of the 1-neighbors of @a aPoint .
     *
     * @param aPoint the point for which the distance is computed
     *
     * @return the distance value at @a aPoint.
     *
     */
    Value operator() (const Point& aPoint);

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    // ----------------------- Internals -------------------------------------

  private: 

    /**
     * Returns the LInf-distance at some point, 
     * knowing the distance of its neighbors
     * 
     * @param aValueList  the distance of (some of) the neighbors
     * @return the computed distance.
     */
    Value compute(Values& aValueList) const; 

  }; 

  /////////////////////////////////////////////////////////////////////////////
  // template class L1FirstOrderLocalDistance
  /**
   * Description of template class 'L1FirstOrderLocalDistance' <p>
   * \brief Aim: Class for the computation of the L1-distance
   * at some point p, from the available distance values of some points 
   * lying in the 1-neighborhood of p (ie. points at a L1-distance to p
   * equal to 1). 
   *
   * The computed value is merely the minimum over all
   * the available distance values in the 1-neighborhood of p, 
   * plus one.  
   *
   * It is a model of CPointFunctor.
   *
   * @tparam TImage model of CImage used for the mapping point-distance value
   * @tparam TSet model of CDigitalSet for storing points whose distance value is known
   */
  template <typename TImage, typename TSet>
  class L1FirstOrderLocalDistance
  {
    // ----------------------- Types ------------------------------
  public:


    /// image
    BOOST_CONCEPT_ASSERT(( CImage<TImage> ));
    typedef TImage Image;
    typedef typename Image::Point Point;
    typedef typename Image::Value Value; 

    /// set
    BOOST_CONCEPT_ASSERT(( CDigitalSet<TSet> ));
    typedef TSet Set;
    BOOST_STATIC_ASSERT(( boost::is_same< Point, typename TSet::Point >::value ));

  private: 

    typedef std::vector<Value> Values; 

    // ----------------------- Data -------------------------------------
  public: 
    /// Aliasing pointer on the underlying image
    Image* myImgPtr; 
    /// Aliasing pointer on the underlying set
    Set* mySetPtr; 

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Constructor from an image and a set. 
     * NB: only pointers are stored
     *
     * @param aImg any distance map
     * @param aSet any digital set
     */
    L1FirstOrderLocalDistance(Image& aImg, TSet& aSet);

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    L1FirstOrderLocalDistance ( const L1FirstOrderLocalDistance & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    L1FirstOrderLocalDistance & operator= ( const L1FirstOrderLocalDistance & other); 

    /**
     * Destructor.
     * Does nothing.
     */
    ~L1FirstOrderLocalDistance(); 

    /** 
     * L1-distance computation at @a aPoint , 
     * from the available distance values
     * of the 1-neighbors of @a aPoint .
     *
     * @param aPoint the point for which the distance is computed
     *
     * @return the distance value at @a aPoint.
     */
    Value operator() (const Point& aPoint);

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    // ----------------------- Internals -------------------------------------

  private: 

    /**
     * Returns the L1-distance at some point, 
     * knowing the distance of its neighbors
     * 
     * @param aValueList  the distance of (some of) the neighbors
     * @return the computed distance.
     */
    Value compute(Values& aValueList) const; 

  }; 

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/FMMPointFunctors.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FMMPointFunctors_h

#undef FMMPointFunctors_RECURSES
#endif // else defined(FMMPointFunctors_RECURSES)
