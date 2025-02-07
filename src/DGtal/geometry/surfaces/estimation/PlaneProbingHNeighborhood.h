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
 * @file
 * @author Jocelyn Meyron (\c jocelyn.meyron@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2020/12/04
 *
 * Header file for module PlaneProbingHNeighborhood.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PlaneProbingHNeighborhood_RECURSES)
#error Recursive header files inclusion detected in PlaneProbingHNeighborhood.h
#else // defined(PlaneProbingHNeighborhood_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PlaneProbingHNeighborhood_RECURSES

#if !defined PlaneProbingHNeighborhood_h
/** Prevents repeated inclusion of headers. */
#define PlaneProbingHNeighborhood_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/helpers/PlaneProbingEstimatorHelper.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingNeighborhood.h"
#include "DGtal/kernel/CPointPredicate.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PlaneProbingHNeighborhood
  /**
   * Description of template class 'PlaneProbingHNeighborhood' <p>
   * \brief Aim: Represent a way to probe the H-neighborhood.
   *
   * \tparam TPredicate the probing predicate, a model of concepts::CPointPredicate.
   */
  template <concepts::CPointPredicate TPredicate>
  class PlaneProbingHNeighborhood : public PlaneProbingNeighborhood<TPredicate>
  {
    // ----------------------- Public types ------------------------------
  public:
      using Predicate           = TPredicate;
      using Point               = typename TPredicate::Point;
      using Triangle            = typename PlaneProbingNeighborhood<TPredicate>::Triangle;
      using PointOnProbingRay   = typename PlaneProbingNeighborhood<TPredicate>::PointOnProbingRay;
      using HexagonState        = typename PlaneProbingNeighborhood<TPredicate>::HexagonState;

    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Default constructor.
     */
    PlaneProbingHNeighborhood() = delete;

    /**
     * Constructor.
     *
     * @param aPredicate a probing predicate.
     * @param aQ the fixed point 'q'.
     * @param aM a frame composed of the three vectors.
     */
    PlaneProbingHNeighborhood(Predicate const& aPredicate, Point const& aQ, Triangle const& aM);

    /**
     * Destructor.
     */
    ~PlaneProbingHNeighborhood();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    PlaneProbingHNeighborhood ( const PlaneProbingHNeighborhood & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    PlaneProbingHNeighborhood ( PlaneProbingHNeighborhood && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    PlaneProbingHNeighborhood & operator= ( const PlaneProbingHNeighborhood & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    PlaneProbingHNeighborhood & operator= ( PlaneProbingHNeighborhood && other ) = delete;

    // ----------------------- Plane Probing services ------------------------------
  public:
    virtual HexagonState hexagonState () override;

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
  protected:

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class PlaneProbingHNeighborhood


  /**
   * Overloads 'operator<<' for displaying objects of class 'PlaneProbingHNeighborhood'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'PlaneProbingHNeighborhood' to write.
   * @return the output stream after the writing.
   */
  template <typename TPredicate>
  std::ostream&
  operator<< ( std::ostream & out, const PlaneProbingHNeighborhood<TPredicate> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/PlaneProbingHNeighborhood.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingHNeighborhood_h

#undef PlaneProbingHNeighborhood_RECURSES
#endif // else defined(PlaneProbingHNeighborhood_RECURSES)
