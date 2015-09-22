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
 * @file PointHashFunctions.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2015/06/14
 *
 * This file is part of the DGtal library.
 */

#if defined(PointHashFunctions_RECURSES)
#error Recursive header files inclusion detected in PointHashFunctions.h
#else // defined(PointHashFunctions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PointHashFunctions_RECURSES

#if !defined PointHashFunctions_h
/** Prevents repeated inclusion of headers. */
#define PointHashFunctions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/base/BasicTypes.h"
#include <boost/functional/hash.hpp>
#ifdef WITH_C11
#include <functional>
#endif
//////////////////////////////////////////////////////////////////////////////


/**
 * Extend std and boost namespaces to define a std::hash (or
 * boost::hash) functiono on DGtal::PointVector.
 *
 */


#ifdef WITH_C11
namespace std {
  template <DGtal::Dimension dim,
  typename EuclideanRing,
  typename Container>
  struct hash< DGtal::PointVector<dim,EuclideanRing,Container> >
  {
    size_t operator()(const DGtal::PointVector<dim,EuclideanRing,Container> & p) const
    {
      return boost::hash_range(p.begin(), p.end());
    }
  };
}
#endif
namespace boost {
  template <DGtal::Dimension dim,
  typename EuclideanRing,
  typename Container>
  struct hash< DGtal::PointVector<dim,EuclideanRing,Container> >
  {
    size_t operator()(const DGtal::PointVector<dim,EuclideanRing,Container> & p) const
    {
      return boost::hash_range(p.begin(), p.end());
    }
  };
}

#ifdef WITH_GMP
#ifdef WITH_C11
namespace std
{
  template <>
  struct hash< DGtal::BigInteger >
  {
    size_t operator()(const DGtal::BigInteger & p) const
    {
      return DGtal::NumberTraits<DGtal::BigInteger>::castToInt64_t( p );
    }
  };
}
#endif
namespace boost
{
  template <>
  struct hash< DGtal::BigInteger >
  {
    size_t operator()(const DGtal::BigInteger & p) const
    {
      return DGtal::NumberTraits<DGtal::BigInteger>::castToInt64_t( p );
    }
  };
}
#endif



#endif // !defined PointHashFunctions_h

#undef PointHashFunctions_RECURSES
#endif // else defined(PointHashFunctions_RECURSES)
