#pragma once

/**
 * @file ImageContainer_map.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/06/15
 *
 * Header file for module ImageContainer_map.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageContainer_map_RECURSES)
#error Recursive header files inclusion detected in ImageContainer_map.h
#else // defined(ImageContainer_map_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageContainer_map_RECURSES

#if !defined ImageContainer_map_h
/** Prevents repeated inclusion of headers. */
#define ImageContainer_map_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <map>

#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class ImageContainer_map
/**
 * Description of class 'ImageContainer_map' <p>
 * Aim:
 */

template <typename TPoint, typename TValue>
class ImageContainer_map: public map<TPoint,TValue>
{
    typedef typename vector<TValue>::size_type TSizeType;

public:

    ImageContainer_map(const TPoint &aPointA,
                       const TPoint &aPointB ) {};

    ~ImageContainer_map() {};

    TValue operator()(const TPoint &aPoint)
    {
	return this->operator[]( aPoint );
    }

    void allocate(const std::size_t aSize) {};
    
private:

};

} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageContainer_map_h

#undef ImageContainer_map_RECURSES
#endif // else defined(ImageContainer_map_RECURSES)
