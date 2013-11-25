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
 * @file StandardDSS6Computer.h 
 * @brief Dynamic recognition of a 3d-digital straight segment (DSS).
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 * @date 2011/06/01
 *
 * Header file for module StandardDSS6Computer.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see testArithDSS3d.cpp
 */

#if defined(StandardDSS6Computer_RECURSES)
#error Recursive header files inclusion detected in StandardDSS6Computer.h
#else // defined(StandardDSS6Computer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define StandardDSS6Computer_RECURSES

#if !defined StandardDSS6Computer_h
/** Prevents repeated inclusion of headers. */
#define StandardDSS6Computer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include "DGtal/base/Exceptions.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/base/ConstIteratorAdapter.h"
#include "DGtal/kernel/BasicPointFunctors.h"

//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{


  /////////////////////////////////////////////////////////////////////////////
  // class StandardDSS6Computer
  /**
   * Description of class 'StandardDSS6Computer' <p>
   * \brief Aim:
   * Dynamic recognition of a 3d-digital straight segment (DSS)

   */
  template <typename TIterator, typename TInteger, int connectivity>
  class StandardDSS6Computer
  {

    // ----------------------- Types ------------------------------
  public:


    //entier
    BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );
    typedef TInteger Integer;


    //requiered types
    typedef TIterator ConstIterator;
    typedef StandardDSS6Computer<ConstIterator,TInteger,connectivity> Self; 
    typedef StandardDSS6Computer<ReverseIterator<ConstIterator>,TInteger,connectivity> Reverse;


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
    typedef DGtal::ArithmeticalDSSComputer<IteratorAdapter,TInteger,connectivity> ArithmeticalDSSComputer2d;
    

    // ----------------------- Standard services ------------------------------
  public:


    /**
     * Default constructor.
     * not valid
     */
    StandardDSS6Computer();

    /**
     * Constructor with initialisation
     * @param it an iterator
     * @see init
     */
    StandardDSS6Computer(const ConstIterator& it);

    /**
     * Initialisation.
     * @param it an iterator
     */
    void init(const ConstIterator& it);


    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    StandardDSS6Computer ( const StandardDSS6Computer & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    StandardDSS6Computer & operator= ( const StandardDSS6Computer & other );

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
    bool operator==( const StandardDSS6Computer & other ) const;

    /**
     * Difference operator.
     * @param other the object to compare with.
     * @return 'false' if equal
     * 'true' otherwise
     */
    bool operator!=( const StandardDSS6Computer & other ) const;

    /**
     * Destructor.
     */
    ~StandardDSS6Computer(){};

    // ----------------------- Interface --------------------------------------
  public:
     

    /**
     * Tests whether the current DSS can be extended at the front. 
     * Computes the parameters of the extended DSS if yes. 
     * with the adding point if true.
     * @return 'true' if yes, 'false' otherwise.
     */
    bool extendFront();
    
    
    /** 
     * Tests whether the 3d DSS can be extended at the front. 
     *
     * @return 'true' if yes, 'false' otherwise
     */   
    bool isExtendableFront();

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

    /**
       @return a const-reference on the arithmetical DSS recognition
       algorithm along the XY plane.
    */
    const ArithmeticalDSSComputer2d & arithmeticalDSS2dXY() const;

    /**
       @return a const-reference on the arithmetical DSS recognition
       algorithm along the XZ plane.
    */
    const ArithmeticalDSSComputer2d & arithmeticalDSS2dXZ() const;

    /**
       @return a const-reference on the arithmetical DSS recognition
       algorithm along the YZ plane.
    */
    const ArithmeticalDSSComputer2d & arithmeticalDSS2dYZ() const;

    /**
       @param i the axis orthogonal to the plane
       @return a const-reference on the arithmetical DSS recognition
       algorithm along the plane orthogonal to the \a i-th axis.
    */
    const ArithmeticalDSSComputer2d & arithmeticalDSS2d( Dimension i ) const;


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
    ArithmeticalDSSComputer2d myXYalgo;
    ArithmeticalDSSComputer2d myXZalgo;
    ArithmeticalDSSComputer2d myYZalgo;

    /// begin and end iterators
    ConstIterator myBegin, myEnd;
    

    // ------------------------- Private Datas --------------------------------

  private:

    
  }; // end of class StandardDSS6Computer



  /**
   * Overloads 'operator<<' for displaying objects of class 'StandardDSS6Computer'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'StandardDSS6Computer' to write.
   * @return the output stream after the writing.
   */
  template <typename TIterator, typename TInteger, int connectivity>
  std::ostream&
  operator<< ( std::ostream & out,  StandardDSS6Computer<TIterator,TInteger,connectivity> & object )
  {
    object.selfDisplay( out);
    return out;
  }


} // namespace DGtal



///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods.
#include "DGtal/geometry/curves/StandardDSS6Computer.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined StandardDSS6Computer_h

#undef StandardDSS6Computer_RECURSES
#endif // else defined(StandardDSS6Computer_RECURSES)
