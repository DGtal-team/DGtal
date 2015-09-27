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
 * @file CLambdaFunctor.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2015/09/12
 *
 * This file is part of the DGtal library.
 */

#if defined(CLambdaFunctor_RECURSES)
#error Recursive header files inclusion detected in CLambdaFunctor.h
#else // defined(CLambdaFunctor_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CLambdaFunctor_RECURSES

#if !defined CLambdaFunctor_h
/** Prevents repeated inclusion of headers. */
#define CLambdaFunctor_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "boost/concept_check.hpp"
#include "DGtal/base/ConceptUtils.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
namespace concepts
{
  
  /////////////////////////////////////////////////////////////////////////////
  // class CLambdaFunctor
  /**
   * Description of \b concept '\b CLambdaFunctor' <p>
   * @ingroup Concepts
   * @brief Aim: Defines the concept describing a lambda function.
   * 
   * A lambda function \lambda() - maps from [0,1] \in \mathbb{R}_+ with F(0) = F(1) = 0
   * and F() > 0 elsewhere and need to satisfy convexity/concavity property.
   * For more information see 
   * J.-O. Lachaud, A. Vialard, and F. de Vieilleville. 
   * Fast, accurate and convergent tangent estimation on digital contours.
   * Image Vision Comput. , 25(10):1572–1587, 2007
   * 
   * @tparam T the type that should be a model of CLambdaFunctor.
   */
  template <typename T>
  struct CLambdaFunctor : boost::DefaultConstructible<T>, boost::CopyConstructible<T>, boost::Assignable<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    BOOST_CONCEPT_USAGE(CLambdaFunctor)
    {
      concepts::ConceptUtils::sameType( d, t.operator()( 10. ) );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    double d;
    T t;
  }; // end of concept CLambdaFunctor

} // namespace concepts
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CLambdaFunctor_h

#undef CLambdaFunctor_RECURSES
#endif // else defined(CLambdaFunctor_RECURSES)
