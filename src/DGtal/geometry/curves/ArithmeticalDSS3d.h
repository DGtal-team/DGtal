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
 * @file ArithmeticalDSS3d.h 
 * @brief Dynamic recognition of a 3d-digital straight segment (DSS).
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 * @date 2011/06/01
 *
 * Header file for module ArithmeticalDSS3d.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see testArithDSS3d.cpp
 */

#if defined(ArithmeticalDSS3d_RECURSES)
#error Recursive header files inclusion detected in ArithmeticalDSS3d.h
#else // defined(ArithmeticalDSS3d_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithmeticalDSS3d_RECURSES

#if !defined ArithmeticalDSS3d_h
/** Prevents repeated inclusion of headers. */
#define ArithmeticalDSS3d_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include "DGtal/base/Exceptions.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/geometry/curves/ArithmeticalDSS.h"
#include "DGtal/base/ConstIteratorAdapter.h"
#include "DGtal/kernel/BasicPointFunctors.h"

//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{


  /////////////////////////////////////////////////////////////////////////////
  // class ArithmeticalDSS3d
  /**
   * Description of class 'ArithmeticalDSS3d' <p>
   * \brief Aim:
   * Dynamic recognition of a 3d-digital straight segment (DSS)

   */
  template <typename TIterator, typename TInteger, int connectivity>
  class ArithmeticalDSS3d
  {

    // ----------------------- Types ------------------------------
  public:


    //entier
    BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );
    typedef TInteger Integer;


    //requiered types
    typedef TIterator ConstIterator;
    typedef ArithmeticalDSS3d<ConstIterator,TInteger,connectivity> Self; 
    typedef ArithmeticalDSS3d<ReverseIterator<ConstIterator>,TInteger,connectivity> Reverse;


    //points and vectors
    typedef typename IteratorCirculatorTraits<ConstIterator>::Value Point3d; 
    typedef typename IteratorCirculatorTraits<ConstIterator>::Value Vector3d;
    typedef typename Point3d::Coordinate Coordinate; 

    typedef DGtal::PointVector<2,Coordinate> Point2d;
    typedef DGtal::PointVector<2,Coordinate> Vector2d;
    
    typedef DGtal::PointVector<2,double> PointD2d; 
    typedef DGtal::PointVector<3,double> PointD3d;
    typedef DGtal::PointVector<3,double> VectorD3d;
        
    // adapters for iterator
    typedef Projector<SpaceND<2,Coordinate> > Projector2d;
    
    typedef ConstIteratorAdapter<ConstIterator,Projector2d,Point2d> IteratorAdapter; 
    

    //2d-arithmeticalDSS recognition algorithm
    typedef DGtal::ArithmeticalDSS<IteratorAdapter,TInteger,connectivity> ArithmeticalDSS2d;
    

    // ----------------------- Standard services ------------------------------
  public:


    /**
     * Default constructor.
     * not valid
     */
    ArithmeticalDSS3d();

    /**
     * Constructor with initialisation
     * @param it an iterator
     * @see init
     */
    ArithmeticalDSS3d(const ConstIterator& it);

    /**
     * Initialisation.
     * @param it an iterator
     */
    void init(const ConstIterator& it);


    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    ArithmeticalDSS3d ( const ArithmeticalDSS3d & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    ArithmeticalDSS3d & operator= ( const ArithmeticalDSS3d & other );

    /** 
     * @return a default-constructed instance of Self.
     */
    Self getSelf() const;

    /** 
     * @return a default-constructed instance of Reverse.
     */
    Reverse getReverse() const;

    

    /**
     * Equality operator.
     * @param other the object to compare with.
     * @return 'true' either if the leaning points perfectly match
     * or if the first leaning points match to the last ones
     * (same DSS scanned in the reverse way) 
     * and 'false' otherwise
     */
    bool operator==( const ArithmeticalDSS3d & other ) const;

    /**
     * Difference operator.
     * @param other the object to compare with.
     * @return 'false' if equal
     * 'true' otherwise
     */
    bool operator!=( const ArithmeticalDSS3d & other ) const;

    /**
     * Destructor.
     */
    ~ArithmeticalDSS3d(){};

    // ----------------------- Interface --------------------------------------
  public:
     

    /**
     * Tests whether the current DSS can be extended at the front. 
     * Computes the parameters of the extended DSS if yes. 
     * with the adding point if true.
     * @return 'true' if yes, 'false' otherwise.
     */
    bool extendForward();
    
    
    /** 
     * Tests whether the 3d DSS can be extended at the front. 
     *
     * @return 'true' if yes, 'false' otherwise
     */   
    bool isExtendableForward();

    // ------------------------- Accessors ------------------------------

    /**
     * Computes the parameters 
     * (direction, intercept, thickness)
     * of the DSS
     * @param direction direction
     * @param intercept intercept
     * @param thickness thickness
     */
    void getParameters(Vector3d& direction, PointD3d& intercept, PointD3d& thickness) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;


    /**
     *  
     * @return begin iterator of the 3d DSS range.
     */
    ConstIterator begin() const;
    /**
     * @return end iterator of the 3d DSS range.
     */
    ConstIterator end() const;


    // ------------------ Display ------------------------------------------

  public:
    
    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) ;

    // ------------------------- Protected Datas ------------------------------
  protected:

    /// projectors
    Projector2d myProjXY, myProjXZ, myProjYZ;

    /// 2d-arithmeticalDSS recognition algorithms
    ArithmeticalDSS2d myXYalgo;
    ArithmeticalDSS2d myXZalgo;
    ArithmeticalDSS2d myYZalgo;

    /// begin and end iterators
    ConstIterator myBegin, myEnd;
    

    // ------------------------- Private Datas --------------------------------

  private:

    
  }; // end of class ArithmeticalDSS3d



  /**
   * Overloads 'operator<<' for displaying objects of class 'ArithmeticalDSS3d'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ArithmeticalDSS3d' to write.
   * @return the output stream after the writing.
   */
  template <typename TIterator, typename TInteger, int connectivity>
  std::ostream&
  operator<< ( std::ostream & out,  ArithmeticalDSS3d<TIterator,TInteger,connectivity> & object )
  {
    object.selfDisplay( out);
    return out;
  }


} // namespace DGtal



///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods.
#include "DGtal/geometry/curves/ArithmeticalDSS3d.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArithmeticalDSS3d_h

#undef ArithmeticalDSS3d_RECURSES
#endif // else defined(ArithmeticalDSS3d_RECURSES)
