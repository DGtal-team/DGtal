#pragma once

/**
 * @file ImageContainer_vector.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/06/15
 *
 * Header file for module ImageContainer_vector.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageContainer_vector_RECURSES)
#error Recursive header files inclusion detected in ImageContainer_vector.h
#else // defined(ImageContainer_vector_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageContainer_vector_RECURSES

#if !defined ImageContainer_vector_h
/** Prevents repeated inclusion of headers. */
#define ImageContainer_vector_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>

#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class ImageContainer_vector
/**
 * Description of class 'ImageContainer_vector' <p>
 * Aim:
 */

template <typename TPoint, typename TValue>
class ImageContainer_vector: public vector<TValue>
{
    typedef typename vector<TValue>::size_type TSizeType;

public:

    ImageContainer_vector(const TPoint &aPointA,
                          const TPoint &aPointB );

    ~ImageContainer_vector();

    TValue operator()(TPoint &aPoint) const;

private:

    /**
     *  Linearized a point and return the vector position.
     * \param aPoint the point to convert to an index
     * \return the index of \param aPoint in the container
     */
    TSizeType linearized(TPoint &aPoint) const;

    TPoint myLowerBound;
    TPoint myUpperBound;
};

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/kernel/images/ImageContainer_vector.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageContainer_vector_h

#undef ImageContainer_vector_RECURSES
#endif // else defined(ImageContainer_vector_RECURSES)
