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
 * Header file for module PlaneProbingRNeighborhood.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PlaneProbingRNeighborhood_RECURSES)
#error Recursive header files inclusion detected in PlaneProbingRNeighborhood.h
#else // defined(PlaneProbingRNeighborhood_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PlaneProbingRNeighborhood_RECURSES

#if !defined PlaneProbingRNeighborhood_h
/** Prevents repeated inclusion of headers. */
#define PlaneProbingRNeighborhood_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingEstimatorCommon.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingNeighborhood.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PlaneProbingRNeighborhood
  /**
   * Description of template class 'PlaneProbingRNeighborhood' <p>
   * \brief Aim:
   *
   * @tparam TPredicate the InPlane predicate.
   */
  template <typename TPredicate>
  class PlaneProbingRNeighborhood : public PlaneProbingNeighborhood<TPredicate>
  {
    // ----------------------- Public types ------------------------------
  public:
      using Predicate    = TPredicate;
      using Point        = typename TPredicate::Point;
      using Triangle     = typename PlaneProbingNeighborhood<TPredicate>::Triangle;
      using Integer      = typename PlaneProbingNeighborhood<TPredicate>::Integer;
      using ProbingRay   = typename PlaneProbingNeighborhood<TPredicate>::ProbingRay;
      using HexagonState = typename PlaneProbingNeighborhood<TPredicate>::HexagonState;

    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Default constructor.
     */
    PlaneProbingRNeighborhood() = delete;

    PlaneProbingRNeighborhood(Predicate const& aPredicate, Point const& aQ, Triangle const& aM);

    /**
     * Destructor.
     */
    ~PlaneProbingRNeighborhood();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    PlaneProbingRNeighborhood ( const PlaneProbingRNeighborhood & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    PlaneProbingRNeighborhood ( PlaneProbingRNeighborhood && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    PlaneProbingRNeighborhood & operator= ( const PlaneProbingRNeighborhood & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    PlaneProbingRNeighborhood & operator= ( PlaneProbingRNeighborhood && other ) = delete;

    // ----------------------- Plane Probing services ------------------------------
  public:
    HexagonState hexagonState ();

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
    ProbingRay closestPointOnRayLogWithPredicate (ProbingRay const& aRay) const;

    ProbingRay closestPointOnRayLinearWithPredicate (ProbingRay const& aRay) const;

    // ------------------------- Private Datas --------------------------------
  private:
    std::vector<ProbingRay> candidates;

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class PlaneProbingRNeighborhood


  /**
   * Overloads 'operator<<' for displaying objects of class 'PlaneProbingRNeighborhood'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'PlaneProbingRNeighborhood' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const PlaneProbingRNeighborhood<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/PlaneProbingRNeighborhood.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingRNeighborhood_h

#undef PlaneProbingRNeighborhood_RECURSES
#endif // else defined(PlaneProbingRNeighborhood_RECURSES)
