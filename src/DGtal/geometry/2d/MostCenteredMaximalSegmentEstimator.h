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
 * @file MostCenteredMaximalSegmentEstimator.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/01/26
 *
 * Header file for module MostCenteredMaximalSegmentEstimator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(MostCenteredMaximalSegmentEstimator_RECURSES)
#error Recursive header files inclusion detected in MostCenteredMaximalSegmentEstimator.h
#else // defined(MostCenteredMaximalSegmentEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define MostCenteredMaximalSegmentEstimator_RECURSES

#if !defined MostCenteredMaximalSegmentEstimator_h
/** Prevents repeated inclusion of headers. */
#define MostCenteredMaximalSegmentEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>

#include "DGtal/geometry/2d/SegmentComputerFunctor.h"
#include "DGtal/geometry/2d/MaximalSegments.h"

#include "DGtal/base/Exceptions.h"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
/** \TODO
* ajouter un myFlagIsInit (aussi dans les decompositions)
* ajouter un getLength() dans les SegmentIterator des decompositions
* mettre au propre les noms de type dans les decompositions (ConstIterator, Self, Reverse, etc.)
*/

  /////////////////////////////////////////////////////////////////////////////
  // template class MostCenteredMaximalSegmentEstimator
  /**
   * Description of template class 'MostCenteredMaximalSegmentEstimator' <p>
   * \brief Aim:Computes a quantity to each element of a range associated to 
   * the most centered maximal segment  
   */
  template <typename SegmentComputer, typename Functor>
  class MostCenteredMaximalSegmentEstimator
  {

    // ----------------------- Types ------------------------------
	public:

    typedef typename SegmentComputer::Iterator ConstIterator;
    typedef typename Functor::Value Quantity;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Default constructor.
     */
    MostCenteredMaximalSegmentEstimator() {};
    /**
     * Constructor.
     * @param h grid size (must be >0).
     * @param itb, begin iterator
     * @param ite, end iterator
     * @param aSegmentComputer
     * @param isClosed true if the input range is closed.
     */
    MostCenteredMaximalSegmentEstimator(
      const double h, 
      const ConstIterator& itb, const ConstIterator& ite,
      const SegmentComputer& aSegmentComputer, 
      const Functor& aFunctor,
      const bool& isClosed);

    /**
     * Destructor.
     */
    ~MostCenteredMaximalSegmentEstimator() {};

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Initialisation.
     * @param h grid size (must be >0).
     * @param itb, begin iterator
     * @param ite, end iterator
     * @param aSegmentComputer
     * @param isClosed true if the input range is viewed as closed.
     */
    void init(
      const double h, 
      const ConstIterator& itb, const ConstIterator& ite,
      const SegmentComputer& aSegmentComputer, 
      const Functor& aFunctor, 
      const bool& isClosed);

    /**
     * @return the estimated quantity at *it
     */
    Quantity eval(const ConstIterator& it);

    /**
     * @return the estimated quantity
     * from itb till ite (exculded)
     */
    template <typename OutputIterator>
    OutputIterator eval(const ConstIterator& itb, const ConstIterator& ite, 
                        OutputIterator result); 


    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  protected:

    // ------------------------- Private Datas --------------------------------
  private:

		double myH; 
		bool myFlagIsClosed;

    Functor myFunctor;

		ConstIterator myBegin;
		ConstIterator myEnd;

    MaximalSegments<SegmentComputer> myMSRange; 

    // ------------------------- Hidden services ------------------------------
  protected:



  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    MostCenteredMaximalSegmentEstimator ( const MostCenteredMaximalSegmentEstimator & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    MostCenteredMaximalSegmentEstimator & operator= ( const MostCenteredMaximalSegmentEstimator & other );


  }; // end of class MostCenteredMaximalSegmentEstimator

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/MostCenteredMaximalSegmentEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MostCenteredMaximalSegmentEstimator_h

#undef MostCenteredMaximalSegmentEstimator_RECURSES
#endif // else defined(MostCenteredMaximalSegmentEstimator_RECURSES)
