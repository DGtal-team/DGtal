#pragma once

/**
 * @file ArithDSS.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/01
 *
 * Header file for module ArithDSS.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ArithDSS_RECURSES)
#error Recursive header files inclusion detected in ArithDSS.h
#else // defined(ArithDSS_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithDSS_RECURSES

#if !defined ArithDSS_h
/** Prevents repeated inclusion of headers. */
#define ArithDSS_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class ArithDSS
/**
 * Description of class 'ArithDSS' <p>
 * \brief Aim:
 * Recognition of a digital straight segment (DSS)
 * based on the arithmetical algorithm of 
 * Debled and Reveilles (1995)
 */
template <typename Domain2D>
class ArithDSS
{
    // ----------------------- Standard services ------------------------------
public:

		
		typedef typename Domain2D::Coordinate Integer;
		//2D point of a domain
		typedef typename Domain2D::Point Point;
		//2D vector of a domain
		typedef typename Domain2D::Vector Vector;


    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    ArithDSS(){};

    /**
     * Destructor.
     */
    ~ArithDSS(){};

    // ----------------------- Interface --------------------------------------
public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) ;
     
    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
		 * \todo implementation
     */
    bool isValid() const;

    /**
		 * Tests whether the union between a point and a DSS
	   * is a DSS. Computes the parameters of the new DSS
     * if true.
     * @return 'true' if the union is a DSS, 'false' otherwise.
     */
    bool add(const Point & aPoint);


    // ------------------------- Protected Datas ------------------------------
protected:

	//parameters of the DSS
	Integer myA, myB, myMu, myOmega;

	//leaning points
	Point myUf, myUl, myLf, myLl;

	//first and last point
	Point myF, myL;

    // ------------------------- Private Datas --------------------------------
	
private:

    // ------------------------- Hidden services ------------------------------
protected:


    /**
		 * Computes the norm of the two components
     * of a 2D vector.
		 * @param x and y, two values. 
     * @return the norm of a 2D vector.
     */
    virtual Integer norm(const Integer& x, const Integer& y) const = 0;

private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    ArithDSS ( const ArithDSS & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    ArithDSS & operator= ( const ArithDSS & other );

    // ------------------------- Internals ------------------------------------
private:

}; // end of class ArithDSS


/**
 * Overloads 'operator<<' for displaying objects of class 'ArithDSS'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'ArithDSS' to write.
 * @return the output stream after the writing.
 */
template<typename Domain2D>
std::ostream&
operator<< ( std::ostream & out,  ArithDSS<Domain2D> & object )
  {
      object.selfDisplay( out);
      return out;
    }


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/geometry/2d/ArithDSS.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArithDSS_h

#undef ArithDSS_RECURSES
#endif // else defined(ArithDSS_RECURSES)
