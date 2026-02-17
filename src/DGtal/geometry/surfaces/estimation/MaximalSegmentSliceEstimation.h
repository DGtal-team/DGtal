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
 * @date 2020/12/15
 *
 * Header file for module MaximalSegmentSliceEstimation.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(MaximalSegmentSliceEstimation_RECURSES)
#error Recursive header files inclusion detected in MaximalSegmentSliceEstimation.h
#else // defined(MaximalSegmentSliceEstimation_RECURSES)
/** Prevents recursive inclusion of headers. */
#define MaximalSegmentSliceEstimation_RECURSES

#if !defined MaximalSegmentSliceEstimation_h
/** Prevents repeated inclusion of headers. */
#define MaximalSegmentSliceEstimation_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/ArithmeticalDSSComputerOnSurfels.h"
#include "DGtal/topology/DigitalSurface2DSlice.h"
#include "DGtal/topology/CDigitalSurfaceContainer.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class MaximalSegmentSliceEstimation
  /**
   * Description of template class 'MaximalSegmentSliceEstimation' <p>
   * \brief Aim:
   *
   * @tparam TSurface the digital surface type.
   *
   * \b Models: A MaximalSegmentSliceEstimation is a model of concepts::CSurfelLocalEstimator and concepts::CDigitalSurfaceLocalEstimator.
   */
  template <typename TSurface>
  class MaximalSegmentSliceEstimation
  {
    BOOST_CONCEPT_ASSERT(( concepts::CDigitalSurfaceContainer<typename TSurface::DigitalSurfaceContainer> ));

    // ----------------------- Public types ------------------------------
  public:
      using Surface      = TSurface;
      using Tracker      = typename Surface::DigitalSurfaceTracker;
      using SurfaceSlice = DigitalSurface2DSlice<Tracker>;
      using Scalar       = double;

      // -------------------------------------- other types ----------------------------
      using KSpace     = typename Surface::KSpace;
      using SCell      = typename KSpace::SCell;
      using Cell       = typename KSpace::Cell;
      using Point      = typename Surface::Point;
      using Vector     = Point;
      using Integer    = typename Point::Coordinate;
      using Space      = typename KSpace::Space;
      using RealPoint  = typename Space::RealPoint;
      using RealVector = RealPoint;

      // ----------------------- model of CDigitalSurfaceLocalEstimator ----------------
      using Surfel   = typename Surface::Surfel;
      using Quantity = RealVector;

    // ----------------------- Private types ------------------------------
  private:
      using Point2      = PointVector<2, Integer>;
      using RealPoint2  = PointVector<2, double>;
      using Container   = std::deque<SCell>;
      using Iterator    = typename Container::const_iterator;
      using DSSComputer = ArithmeticalDSSComputerOnSurfels<KSpace, Circulator<Iterator>, Integer>;

    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Default constructor.
     */
    MaximalSegmentSliceEstimation();

    /**
     * Constructor.
     *
     * @param aSurface a digiral surface.
     */
    MaximalSegmentSliceEstimation(ConstAlias<Surface> aSurface);

    /**
     * Destructor.
     */
    ~MaximalSegmentSliceEstimation();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    MaximalSegmentSliceEstimation ( const MaximalSegmentSliceEstimation & other );

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    MaximalSegmentSliceEstimation & operator= ( const MaximalSegmentSliceEstimation & other );

    // ----------------- model of CSurfelLocalEstimator -----------------------
  public:
    /**
     * Initializes the estimator (in this case, do nothing apart from storing the gridstep).
     *
     * @param h the grdstep.
     * @param itb an iterator on the start of the range of surfels.
     * @param ite a past-the-end iterator of the range of surfels.
     */
    template < typename SurfelConstIterator >
    void init (Scalar const& h, SurfelConstIterator itb, SurfelConstIterator ite);

    /**
     * Estimates the quantity on a surfel.
     *
     * @param it an iterator whose value type is a Surfel.
     * @return the estimated quantity.
     */
    template < typename SurfelConstIterator >
    Quantity eval (SurfelConstIterator it) const;

    /**
     * Estimates the quantity on a range of surfels.
     *
     * @param itb an iterator on the start of the range of surfels.
     * @param ite a past-the-end iterator of the range of surfels.
     * @param out an output iterator to store the results.
     * @return the modified output iterator.
     */
    template < typename SurfelConstIterator, typename OutputIterator >
    OutputIterator eval (SurfelConstIterator itb, SurfelConstIterator ite, OutputIterator out) const;

    /**
     * @return the gridstep.
     */
    Scalar h () const;

    // --------------- model of CDigitalSurfaceLocalEstimator ------------------
  public:
    void attach (ConstAlias<Surface> aSurface);

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

    // ------------------------- Protected Data ------------------------------
  protected:

    // ------------------------- Private Data --------------------------------
  private:
    CountedConstPtrOrConstPtr<Surface> mySurface; /**< A pointer to the digital surface. */
    Scalar myH; /**< The gridstep */

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:
    /**
     * Khalimsky space associated to the digital surface.
     *
     * @return a const reference to the Khalimsky space.
     */
    KSpace const& space () const;

    /**
     * Computes the extremal points of a set of 2D digital points using maximal segments.
     *
     * @param aSlice a slice of the digital surface.
     * @param aProjectDir1 the first direction of projection.
     * @param aProjectDir2 the second direction of projection.
     * @return the two extremities of the maximal segment on the left and on the right.
     */
    std::pair<Point2, Point2> getExtremalPoints (SurfaceSlice const& aSlice,
                                                 Dimension const& aProjectDir1, Dimension const& aProjectDir2) const;

    /**
     * @param aSurfel a surfel.
     * @return the trivial normal vector of a surfel.
     */
    Vector trivialNormal (Surfel const& aSurfel) const;
  }; // end of class MaximalSegmentSliceEstimation


  /**
   * Overloads 'operator<<' for displaying objects of class 'MaximalSegmentSliceEstimation'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'MaximalSegmentSliceEstimation' to write.
   * @return the output stream after the writing.
   */
  template <typename TSurface>
  std::ostream&
  operator<< ( std::ostream & out, const MaximalSegmentSliceEstimation<TSurface> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/MaximalSegmentSliceEstimation.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MaximalSegmentSliceEstimation_h

#undef MaximalSegmentSliceEstimation_RECURSES
#endif // else defined(MaximalSegmentSliceEstimation_RECURSES)
