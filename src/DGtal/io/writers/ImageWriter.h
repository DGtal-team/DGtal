#pragma once

/**
 * @file ImageWriter.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/16
 *
 * Header file for module ImageWriter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageWriter_RECURSES)
#error Recursive header files inclusion detected in ImageWriter.h
#else // defined(ImageWriter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageWriter_RECURSES

#if !defined ImageWriter_h
/** Prevents repeated inclusion of headers. */
#define ImageWriter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ImageWriter
  /**
   * Description of template class 'ImageWriter' <p>
   * \brief Aim: Implements writers associated to a generic DGtal Image
   */
  template <typename TImageModel>
  class ImageWriter
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~ImageWriter();

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
    ImageWriter();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    ImageWriter ( const ImageWriter & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    ImageWriter & operator= ( const ImageWriter & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ImageWriter


  /**
   * Overloads 'operator<<' for displaying objects of class 'ImageWriter'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ImageWriter' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const ImageWriter<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/writers/ImageWriter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageWriter_h

#undef ImageWriter_RECURSES
#endif // else defined(ImageWriter_RECURSES)
