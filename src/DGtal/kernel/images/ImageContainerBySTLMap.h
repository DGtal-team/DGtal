#pragma once

/**
 * @file ImageContainerBySTLMap.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/06/15
 *
 * Header file for module ImageContainerBySTLMap.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageContainerBySTLMap_RECURSES)
#error Recursive header files inclusion detected in ImageContainerBySTLMap.h
#else // defined(ImageContainerBySTLMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageContainerBySTLMap_RECURSES

#if !defined ImageContainerBySTLMap_h
/** Prevents repeated inclusion of headers. */
#define ImageContainerBySTLMap_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <map>

#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class ImageContainerBySTLMap
/**
 * Description of class 'ImageContainerBySTLMap' <p>
 * Aim:
 * \todo Documentation
 */

template <typename TPoint, typename TValue>
class ImageContainerBySTLMap: public map<TPoint,TValue>
{

public:
    typedef typename map<TPoint,TValue>::size_type TSizeType;
    typedef typename map<TPoint,TValue>::iterator Iterator;
    typedef typename map<TPoint,TValue>::const_iterator ConstIterator;

    ///\todo create span iterators
    class SpanIterator: public Iterator
    {
    };

    ImageContainerBySTLMap(const TPoint &aPointA,
                       const TPoint &aPointB ) {};

    ~ImageContainerBySTLMap() {};

    TValue operator()(const TPoint &aPoint) throw( std::bad_alloc )
    {
        Iterator it = this->find( aPoint );
        if ( it == this->end() )
            throw std::bad_alloc();
        else
            return this->operator[]( aPoint );
    }


    TValue operator()(const Iterator &it) throw( std::bad_alloc )
    {
        if ( it == this->end() )
            throw std::bad_alloc();
        else
            return (*it).second;
    }
    

    void setValue(const TPoint &aPoint, const TValue &aValue)
    {
      this->operator[]( aPoint ) ;
    }


    void setValue(SpanIterator &it, const TValue &aValue)
    {
       ASSERT("NOT-YET-IMPLEMENTED");
    }

    void setValue(Iterator &it, const TValue &aValue)
    {
        (*it) = aValue;
    }


    void allocate(const std::size_t aSize) {};

private:

};

} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageContainerBySTLMap_h

#undef ImageContainerBySTLMap_RECURSES
#endif // else defined(ImageContainerBySTLMap_RECURSES)
