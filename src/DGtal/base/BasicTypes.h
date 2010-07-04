#pragma once

/**
 * @file BasicTypes.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/05/15
 *
 * Header file for module BasicTypes.cpp.
 *
 * This file contains the definition of basic types.
 *
 * This file is part of the DGtal library.
 */

#if defined(BasicTypes_RECURSES)
#error Recursive header files inclusion detected in BasicTypes.h
#else // defined(BasicTypes_RECURSES)
/** Prevents recursive inclusion of headers. */
#define BasicTypes_RECURSES

#if !defined BasicTypes_h
/** Prevents repeated inclusion of headers. */
#define BasicTypes_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <cstdlib>
#include <iostream>
#include <boost/cstdint.hpp>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////



namespace DGtal
{
  typedef std::size_t size_t;
  typedef unsigned int uint;

  typedef boost::uint16_t uint16_t;
  typedef boost::uint32_t uint32_t;
  typedef boost::uint64_t uint64_t;
  
  typedef boost::int16_t int16_t;
  typedef boost::int32_t int32_t;
  typedef boost::int64_t int64_t;


/////////////////////////////////////////////////////////////////////////////
// class BasicTypes
/**
 * Description of class 'BasicTypes' <p>
 * Aim: This class tests the correctness of basic types.
 * @todo Check all defined basic types.
 */
class BasicTypes
{
    // ----------------------- Standard services ------------------------------
public:

    /**
     * Destructor.
     */
    ~BasicTypes();

    // ----------------------- Interface --------------------------------------
public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
private:
    // ------------------------- Private Datas --------------------------------
private:

    // ------------------------- Hidden services ------------------------------
protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    BasicTypes();

private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    BasicTypes ( const BasicTypes & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    BasicTypes & operator= ( const BasicTypes & other );

    // ------------------------- Internals ------------------------------------
private:

}; // end of class BasicTypes


/**
 * Overloads 'operator<<' for displaying objects of class 'BasicTypes'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'BasicTypes' to write.
 * @return the output stream after the writing.
 */
inline
  std::ostream&
  operator<< ( std::ostream & out, const BasicTypes & object )
 {
   object.selfDisplay( out );
   return out;
  }


} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined BasicTypes_h

#undef BasicTypes_RECURSES
#endif // else defined(BasicTypes_RECURSES)
