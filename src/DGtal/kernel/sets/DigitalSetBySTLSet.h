#pragma once

/**
 * @file DigitalSetBySTLSet.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et
 * Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/01
 *
 * Header file for module DigitalSetBySTLSet.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalSetBySTLSet_RECURSES)
#error Recursive header files inclusion detected in DigitalSetBySTLSet.h
#else // defined(DigitalSetBySTLSet_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalSetBySTLSet_RECURSES

#if !defined DigitalSetBySTLSet_h
/** Prevents repeated inclusion of headers. */
#define DigitalSetBySTLSet_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DigitalSetBySTLSet
  /**
   * Description of template class 'DigitalSetBySTLSet' <p>
   * \brief Aim:
   */
  template <typename T>
  class DigitalSetBySTLSet
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~DigitalSetBySTLSet();

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
    DigitalSetBySTLSet();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    DigitalSetBySTLSet ( const DigitalSetBySTLSet & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    DigitalSetBySTLSet & operator= ( const DigitalSetBySTLSet & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class DigitalSetBySTLSet


  /**
   * Overloads 'operator<<' for displaying objects of class 'DigitalSetBySTLSet'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalSetBySTLSet' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalSetBySTLSet<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/sets/DigitalSetBySTLSet.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSetBySTLSet_h

#undef DigitalSetBySTLSet_RECURSES
#endif // else defined(DigitalSetBySTLSet_RECURSES)
