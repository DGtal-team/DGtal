#pragma once

/**
 * @file DigitalSetBySTLVector.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et
 * Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/01
 *
 * Header file for module DigitalSetBySTLVector.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalSetBySTLVector_RECURSES)
#error Recursive header files inclusion detected in DigitalSetBySTLVector.h
#else // defined(DigitalSetBySTLVector_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalSetBySTLVector_RECURSES

#if !defined DigitalSetBySTLVector_h
/** Prevents repeated inclusion of headers. */
#define DigitalSetBySTLVector_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DigitalSetBySTLVector
  /**
   * Description of template class 'DigitalSetBySTLVector' <p>
   * \brief Aim:
   */
  template <typename T>
  class DigitalSetBySTLVector
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~DigitalSetBySTLVector();

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
    DigitalSetBySTLVector();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    DigitalSetBySTLVector ( const DigitalSetBySTLVector & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    DigitalSetBySTLVector & operator= ( const DigitalSetBySTLVector & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class DigitalSetBySTLVector


  /**
   * Overloads 'operator<<' for displaying objects of class 'DigitalSetBySTLVector'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalSetBySTLVector' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalSetBySTLVector<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/sets/DigitalSetBySTLVector.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSetBySTLVector_h

#undef DigitalSetBySTLVector_RECURSES
#endif // else defined(DigitalSetBySTLVector_RECURSES)
