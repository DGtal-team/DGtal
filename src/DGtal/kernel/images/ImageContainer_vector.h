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
 * \todo Documentation
 * Aim:
 */

template <class TPoint, typename TValue>
class ImageContainer_vector: public vector<TValue>
{
public:

    typedef typename vector<TValue>::size_type TSizeType;
    typedef typename vector<TValue>::iterator Iterator;
    typedef typename vector<TValue>::const_iterator ConstIterator;

    ImageContainer_vector(const TPoint &aPointA,
                          const TPoint &aPointB );

    ~ImageContainer_vector();

    TValue operator()(const TPoint &aPoint);

    void allocate(const std::size_t aSize) {
        this->resize( aSize );
    }

private:

    /**
     *  Linearized a point and return the vector position.
     * \param aPoint the point to convert to an index
     * \return the index of \param aPoint in the container
     */
    TSizeType linearized(const TPoint &aPoint) const;

    TPoint myLowerBound;
    TPoint myUpperBound;
};

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/kernel/images/ImageContainer_vector.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageContainer_vector_h

#undef ImageContainer_vector_RECURSES
#endif // else defined(ImageContainer_vector_RECURSES)
