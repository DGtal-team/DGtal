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
#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/topology/DigitalSurface2DSlice.h"
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
   */
  template <typename TSurface>
  class MaximalSegmentSliceEstimation
  {
    // ----------------------- Public types ------------------------------
  public:
      using Surface      = TSurface;
      using Tracker      = typename Surface::DigitalSurfaceTracker;
      using SurfaceSlice = DigitalSurface2DSlice<Tracker>;
      using Scalar       = double;

      // -------------------------------------- other types ----------------------------
      using KSpace    = typename Surface::KSpace;
      using SCell     = typename KSpace::SCell;
      using Cell      = typename KSpace::Cell;
      using Point     = typename Surface::Point;
      using Integer   = typename Point::Coordinate;
      using Space     = typename KSpace::Space;
      using RealPoint = typename Space::RealPoint;

      // ----------------------- model of CDigitalSurfaceLocalEstimator ----------------
      using Surfel   = typename Surface::Surfel;
      struct Quantity {
          RealPoint normal; /**< The estimated normal. */
          std::vector<int> flatDirections; /**< The dimensions that were found to be locally flat. */
      };

    // ----------------------- Private types ------------------------------
  private:
      using Point2      = PointVector<2, Integer>;
      using RealPoint2  = PointVector<2, double>;
      using Container   = std::vector<Point2>;
      using Iterator    = typename Container::const_iterator;
      using DSSComputer = StandardDSS4Computer<Circulator<Iterator>>;

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
     * Move constructor.
     * @param other the object to move.
     */
    MaximalSegmentSliceEstimation ( MaximalSegmentSliceEstimation && other );

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    MaximalSegmentSliceEstimation & operator= ( const MaximalSegmentSliceEstimation & other );

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    MaximalSegmentSliceEstimation & operator= ( MaximalSegmentSliceEstimation && other );

    // ----------------- model of CSurfelLocalEstimator -----------------------
  public:
    template < typename SurfelConstIterator >
    void init (Scalar const& h, SurfelConstIterator itb, SurfelConstIterator ite);

    template < typename SurfelConstIterator >
    Quantity eval (SurfelConstIterator it);

    template < typename SurfelConstIterator, typename OutputIterator >
    OutputIterator eval (SurfelConstIterator itb, SurfelConstIterator ite, OutputIterator out);

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

    // ------------------------- Protected Datas ------------------------------
  protected:

    // ------------------------- Private Datas --------------------------------
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
     * Computes the center point of a given surfel.
     *
     * @param aSurfel a surfel.
     * @return the center point.
     */
    RealPoint centerSurfel (Surfel const& aSurfel) const;

    /**
     * Returns the two points of the surfel 'aSurfel' located on the linel on direction (aOtherDir, +).
     *
     * @param aSurfel a surfel.
     * @param aOtherDir the direction in which we want to look at.
     * @return the two points on the linel in this direction.
     */
    std::pair<Point, Point> trackingPointsSurfel (Surfel const& aSurfel, Dimension const& aOtherDir) const;

    /**
     * Returns the 2D coordinates of a 3D point in the plane defined by two directions and an origin point.
     *
     * @param aDir1 a direction.
     * @param aDir2 an other direction.
     * @param aDirOrth the orthogonal direction.
     * @param aOrigin a point on the plane.
     * @param aPoint the point to project.
     * @return the projected 2D point.
     */
    RealPoint2 projectInPlane (Dimension const& aDir1, Dimension const& aDir2,
                               Dimension const& aDirOrth, Point const& aOrigin,
                               Point const& aPoint) const;

    /**
     * Computes the set of points contained on the slice starting from surfel aSurfel
     * in direction aSliceDir, projected on the plane of the surfel.
     *
     * @param aSurfel a surfel.
     * @param aSliceDir the direction of the slice.
     * @param aOtherDir the other direction.
     * @param aOrthDir the direction orthogonal to aSliceDir and aOtherDir.
     * @return the set of 2D digital points.
     */
    Container slicePoints (Surfel const& aSurfel,
                           Dimension const& aSliceDir,
                           Dimension const& aOtherDir,
                           Dimension const& aOrthDir) const;

    /**
     * Computes the extremal points of a set of 2D digital points using maximal segments.
     *
     * @param aPoints a set of 2D digital points.
     * @return the two extremities of the maximal segment on the left and on the right.
     */
    std::pair<Point2, Point2> getExtremalPoints (Container const& aPoints) const;

    /**
     * Computes the trivial normal of a surfel.
     *
     * @param aSurfel a surfel.
     * @return the trivial normal.
     */
    Point trivialNormal (Surfel const& aSurfel) const;
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
