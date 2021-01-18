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
 * @date 2020/12/07
 *
 * Header file for module PlaneProbingDigitalSurfaceLocalEstimator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PlaneProbingDigitalSurfaceLocalEstimator_RECURSES)
#error Recursive header files inclusion detected in PlaneProbingDigitalSurfaceLocalEstimator.h
#else // defined(PlaneProbingDigitalSurfaceLocalEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PlaneProbingDigitalSurfaceLocalEstimator_RECURSES

#if !defined PlaneProbingDigitalSurfaceLocalEstimator_h
/** Prevents repeated inclusion of headers. */
#define PlaneProbingDigitalSurfaceLocalEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <unordered_map>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/DigitalSurfacePredicate.h"
#include "DGtal/geometry/surfaces/estimation/MaximalSegmentSliceEstimation.h"
#include "DGtal/topology/KhalimskyCellHashFunctions.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PlaneProbingDigitalSurfaceLocalEstimator
  /**
   * Description of template class 'PlaneProbingDigitalSurfaceLocalEstimator' <p>
   * \brief Aim: Adapt a plane-probing estimator on a digital surface to estimate normal vectors.
   *
   * This class uses a plane-probing algorithm (whose type is given by the template parameter TProbingAlgorithm) to estimate
   * normal vectors on a digital surface per surfel.
   *
   * @tparam TSurface the digital surface type.
   * @tparam TProbingAlgorithm the probing algorithm (see \ref PlaneProbingTetrahedronEstimator or PlaneProbingParallelepipedEstimator).
   *
   * \b Models: A PlaneProbingDigitalSurfaceLocalEstimator is a model of concepts::CSurfelLocalEstimator and concepts::CDigitalSurfaceLocalEstimator.
   */
  template <typename TSurface, typename TProbingAlgorithm>
  class PlaneProbingDigitalSurfaceLocalEstimator
  {
    // ----------------------- Public types ------------------------------
  public:
      using Surface          = TSurface;
      using ProbingAlgorithm = TProbingAlgorithm;
      using Point            = typename ProbingAlgorithm::Point;
      using Scalar           = double;

      /**
       * A probing frame represents a 3D orthogonal frame, that is internally used
       * to define the starting position of a plane-probing algorithm.
       */
      struct ProbingFrame
      {
          Point p; /**< The base point/ */
          Point b1; /**< The first base vector. */
          Point b2; /**< The second base vector. */
          Point normal; /**< A vector that is orthogonal to b1 and b2. */

          /**
           * @return a copy of the current probing frame rotated clockwise.
           */
          ProbingFrame rotatedCopy () const
          {
              Point newP      = p + b1,
                    newB1     = b2,
                    newB2     = -b1,
                    newNormal = newB1.crossProduct(newB2);

              return  { newP, newB1, newB2, newNormal };
          }

          /**
           * @return the 'shift' vector of the frame i.e. the octant that it represents.
           */
          Point shift () const
          {
              return b1 + b2 + normal;
          }
      };

      using Predicate        = DigitalSurfacePredicate<Surface>;
      using ProbingFactory   = std::function<ProbingAlgorithm*(const ProbingFrame&, Predicate const&)>;
      using PreEstimation    = MaximalSegmentSliceEstimation<Surface>;
      using ProbingRay       = typename ProbingAlgorithm::ProbingRay;

      // ----------------------- model of CDigitalSurfaceLocalEstimator ----------------
      using Surfel   = typename Surface::Surfel;
      using Quantity = typename ProbingAlgorithm::Quantity;

      // -------------------------------------- other types ----------------------------
      using KSpace    = typename Surface::KSpace;
      using SCell     = typename KSpace::SCell;
      using Cell      = typename KSpace::Cell;
      using Space     = typename KSpace::Space;
      using RealPoint = typename Space::RealPoint;

    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Default constructor.
     */
    PlaneProbingDigitalSurfaceLocalEstimator();

    /*
     * Constructor.
     *
     * @param aSurface a digital surface.
     */
    PlaneProbingDigitalSurfaceLocalEstimator(ConstAlias<Surface> aSurface);

    /*
     * Constructor.
     *
     * @param aProbingFactory functor to produce a plane-probing estimator from a frame and a predicate.
     * @param aPreEstimations map of pre-estimated normal vectors (empty by default) to help choose the correct octant.
     * @param aVerbose a boolean indicating the level of verbosity.
     */
    PlaneProbingDigitalSurfaceLocalEstimator(ProbingFactory const& aProbingFactory,
                                             std::unordered_map<Surfel, RealPoint> const& aPreEstimations = {},
                                             bool aVerbose = false);

    /*
     * Constructor.
     *
     * @param aSurface a digital surface.
     * @param aProbingFactory functor to produce a plane-probing estimator from a frame and a predicate.
     * @param aPreEstimations map of pre-estimated normal vectors (empty by default) to help choose the correct octant.
     * @param aVerbose a boolean indicating the level of verbosity.
     */
    PlaneProbingDigitalSurfaceLocalEstimator(ConstAlias<Surface> aSurface,
                                             ProbingFactory const& aProbingFactory,
                                             std::unordered_map<Surfel, RealPoint> const& aPreEstimations = {},
                                             bool aVerbose = false);

    /**
     * Destructor.
     */
    ~PlaneProbingDigitalSurfaceLocalEstimator();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    PlaneProbingDigitalSurfaceLocalEstimator ( const PlaneProbingDigitalSurfaceLocalEstimator & other );

    /**
     * Move constructor.
     * @param other the object to move.
     */
    PlaneProbingDigitalSurfaceLocalEstimator ( PlaneProbingDigitalSurfaceLocalEstimator && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    PlaneProbingDigitalSurfaceLocalEstimator & operator= ( const PlaneProbingDigitalSurfaceLocalEstimator & other );

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    PlaneProbingDigitalSurfaceLocalEstimator & operator= ( PlaneProbingDigitalSurfaceLocalEstimator && other ) = delete;

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

    void setParams (ProbingFactory const& aProbingFactory,
                    std::unordered_map<Surfel, RealPoint> const& aPreEstimations = {},
                    bool aVerbose = false);

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

    /**
     * Get the pre-estimation on a given normal.
     *
     * @param aSurfel a surfel.
     */
    RealPoint getPreEstimation (Surfel const& s) const;

    // ------------------------- Protected Datas ------------------------------
  protected:

    // ------------------------- Private Datas --------------------------------
  private:
    ProbingAlgorithm* myProbingAlgorithm = nullptr;
    bool myH;
    CountedConstPtrOrConstPtr<Surface> mySurface;
    Predicate myPredicate;
    PreEstimation myPreEstimationEstimator;
    ProbingFactory myProbingFactory;
    std::unordered_map<Surfel, RealPoint> myPreEstimations;
    bool myVerbose;

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:

    ProbingFrame probingFrameFromSurfel (Surfel const& aSurfel) const;

    ProbingFrame probingFrameWithPreEstimation (ProbingFrame const& aInitialFrame, RealPoint const& aPreEstimation) const;

    static int signComponent (double x)
    {
        return (x >= 0) ? 1 : -1;
    }

    static std::vector<int> findZeros (RealPoint const& p)
    {
        std::vector<int> zeros;

        for (int i = 0; i < 3; ++i)
        {
            if (p[i] == 0)
            {
                zeros.push_back(i);
            }
        }

        return zeros;
    }

    static std::vector<ProbingRay> getProbingRaysOneFlatDirection (int aIndex)
    {
        if (aIndex == 0)
        {
            return { ProbingRay({ 2, 1, 0 }), ProbingRay({ 1, 2, 0 }) };
        }
        else if (aIndex == 1)
        {
            return { ProbingRay({ 0, 2, 1 }), ProbingRay({ 2, 0, 1 }) };
        }
        else
        {
            assert(aIndex == 2);
            return { ProbingRay({ 1, 0, 2 }), ProbingRay({ 0, 1, 2 }) };
        }
    }

    Point getNormalOneFlatDirection (int aIndex) const
    {
        int im1 = (aIndex - 1 + 3) % 3,
            im2 = (aIndex - 2 + 3) % 3;

        return myProbingAlgorithm->m(im1).crossProduct(myProbingAlgorithm->m(aIndex)) +
            myProbingAlgorithm->m(aIndex).crossProduct(myProbingAlgorithm->m(im2));
    }
  }; // end of class PlaneProbingDigitalSurfaceLocalEstimator


  /**
   * Overloads 'operator<<' for displaying objects of class 'PlaneProbingDigitalSurfaceLocalEstimator'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'PlaneProbingDigitalSurfaceLocalEstimator' to write.
   * @return the output stream after the writing.
   */
  template < typename TSurface, typename TProbingAlgorithm >
  std::ostream&
  operator<< ( std::ostream & out, const PlaneProbingDigitalSurfaceLocalEstimator<TSurface, TProbingAlgorithm> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/PlaneProbingDigitalSurfaceLocalEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingDigitalSurfaceLocalEstimator_h

#undef PlaneProbingDigitalSurfaceLocalEstimator_RECURSES
#endif // else defined(PlaneProbingDigitalSurfaceLocalEstimator_RECURSES)
