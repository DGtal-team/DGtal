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
 * @file CNormalVectorEstimator.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/04/26
 *
 * Header file for concept CNormalVectorEstimator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CNormalVectorEstimator_RECURSES)
#error Recursive header files inclusion detected in CNormalVectorEstimator.h
#else // defined(CNormalVectorEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CNormalVectorEstimator_RECURSES

#if !defined CNormalVectorEstimator_h
/** Prevents repeated inclusion of headers. */
#define CNormalVectorEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <boost/concept_archetype.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
//////////////////////////////////////////////////////////////////////////////

// @since 0.8 In DGtal::concepts
namespace DGtal {
  namespace concepts {

/////////////////////////////////////////////////////////////////////////////
// class CNormalVectorEstimator
/**
Description of \b concept '\b CNormalVectorEstimator' <p>
@ingroup Concepts
@brief Aim: Represents the concept of estimator of normal vector along digital surfaces.

# Refinement of

# Associated types
- Surface : the type of digital surface (must be some DigitalSurface<X>)
- ConstIterator : an iterator on the signed cells of the digital surface (same as Surface::ConstIterator).
- SCell : the type of signed cell (for surfels along the digital surface) (same as Surface::SCell).
- Quantity: the type representing normal vectors.

# Notation
- \e X : A type that is a model of CNormalVectorEstimator
- \e x : object of type X
- \e sc : object of type SCell
- \e out_it : an \e OutputIterator on \e Quantity
# Definitions

# Valid expressions and semantics

| Name                      | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|---------------------------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
| Accessor to surface       |\e x.surface()|                 | const \e Surface &|             | returns a reference to the digital surface | | |
| Local normal vector estimation  |\e x.eval(\e sc)|         | \e Quantity    |              | returns the estimation of the normal vector at the signed cell \e sc | | |
| Global normal vector estimation |\c template \<OutputIterator\>\e x.evalAll(\e out_it)|  | \e OutputIterator | | outputs the estimation of the normal vector field of the whole surface | | |

# Invariants

# Models

- LocalConvolutionNormalVectorEstimator

# Notes

@tparam T the type that should be a model of CNormalVectorEstimator.
 */
template <typename T>
concept CNormalVectorEstimator =  
  CCellularGridSpaceND<typename T::Surface::KSpace> && 
  ConceptUtils::InputIterator<typename T::ConstIterator> && 
  std::same_as<typename T::SCell, typename T::Surface::SCell> && 
  requires(T myX, typename T::SCell mySCell, typename boost::output_iterator_archetype<typename T::Quantity> myOutIt)
  {
      { myX.surface() } -> std::same_as<const typename T::Surface&>;
      { myX.eval(mySCell) } -> std::same_as<typename T::Quantity>;
      { myX.evalAll(myOutIt) } -> std::same_as<typename boost::output_iterator_archetype<typename T::Quantity>>;
  };
} // namespace concepts
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CNormalVectorEstimator_h

#undef CNormalVectorEstimator_RECURSES
#endif // else defined(CNormalVectorEstimator_RECURSES)
