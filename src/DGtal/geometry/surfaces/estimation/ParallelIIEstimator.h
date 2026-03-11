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

#include <DGtal/geometry/surfaces/estimation/DomainSplitter.h>
#include <DGtal/topology/helpers/Surfaces.h>

// We require openmp for this estimator, even though it could run without
#include <omp.h>

namespace DGtal
{
  /**
   * @brief Run an Integral Invariant estimator in parallel
   *
   * This class is meant as an almost perfect replacement of 
   * other IIEstimator. The only difference is the constructor
   * which needs the number of threads.
   *
   * TODO: Value to control output (ie. value + loc, just value or ordered value)
   * TODO: Surface construction on subdomain fonctor ?
   *
   * @tparam TEstimator The model of Estimator
   * @tparam TSplitter The function to split the domain
   */
  template<class TEstimator, typename TSplitter> 
  class ParallelIIEstimator
  {
  public:
    using Estimator = TEstimator;
    using Splitter = TSplitter;
    using Domain = typename TEstimator::Domain;
    using Scalar = typename TEstimator::Scalar;

    using KSpace = typename TEstimator::KSpace;
    using PointPredicate = typename TEstimator::PointPredicate;
    using Surfel = typename KSpace::Surfel;
    using SurfelSet = typename KSpace::SurfelSet;
    using EstimatorQuantity = typename TEstimator::Quantity;
    using Quantity = EstimatorQuantity;
    
    // Building the surface
    using Boundary = LightImplicitDigitalSurface<KSpace, PointPredicate>;
    using Surface = DigitalSurface<Boundary>;
    using Visitor = DepthFirstVisitor<Surface>;
    using VisitorRange = GraphVisitorRange<Visitor>;
  
   
    /**
     * @brief Constructor
     *
     * @tparam Args The estimator constructor arguments
     *
     * @param nbThread The number of thread to run in parallel. -1 means as many as possible
     * @param args Constructor arguments to underlying estimators
     */
    template<typename... Args>
    ParallelIIEstimator(int32_t nbThread, Args&&... args); 
   
    /**
     * Clears the object. It is now invalid.
     */
    void clear();

    /// @return the grid step.
    Scalar h() const;

    /**
     * Attach a shape, defined as a functor spel -> boolean
     *
     * @param[in] K the cellular grid space in which the shape is defined.
     * @param aPointPredicate the shape of interest. The alias can be secured
     * if a some counted pointer is handed.
    */
    void attach(ConstAlias<KSpace> K, 
                ConstAlias<PointPredicate> pp);

    /**
     * Set specific parameters: the radius of the ball.
     *
     * @param[in] dRadius the "digital" radius of the kernel (buy may be non integer).
     */
    void setParams(double dRadius);


    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */ 
     bool isValid() const;

    /**
     * Model of CDigitalSurfaceLocalEstimator. Initialisation.
     *
     * @tparam SurfelConstIterator any model of forward readable iterator on Surfel.
     * @param[in] _h grid size (must be >0).
     * @param[in] ite iterator on the first surfel of the surface.
     * @param[in] itb iterator after the last surfel of the surface.
     */
    template<typename ItA, typename ItB>
    void init(double h_, ItA, ItB);

    /**
     * -- Estimation --
     *
     * Compute the integral invariant volume at surfel *it of
     * a shape, then apply the VolumeFunctor to extract some
     * geometric information.
     *
     * @tparam SurfelConstIterator type of Iterator on a Surfel
     *
     * @param[in] it iterator pointing on the surfel of the shape where
     * we wish to evaluate some geometric information.
     *
     * @return quantity (normal vector) at surfel *it
     */
    template<typename It>
    Quantity eval(It it);
    /**
     * -- Estimation --
     *
     * Compute the integral invariant volume for a range of
     * surfels [itb,ite) on a shape, then apply the
     * VolumeFunctor to extract some geometric information.
     * Return the result on an OutputIterator (param).
     *
     * @tparam OutputIterator type of Iterator of an array of Quantity
     * @tparam SurfelConstIterator type of Iterator on a Surfel
     *
     * @param[in] itb iterator defining the start of the range of surfels
     * where we wish to compute some geometric information.
     *
     * @param[in] ite iterator defining the end of the range of surfels
     * where we wish to compute some geometric information.
     *
     * @param[in] result output iterator of results of the computation.
     * @return the updated output iterator after all outputs.
     */
    template<typename It, typename Oit>
    Oit eval(It itb, It ite, Oit rslt);

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

  private:
    std::vector<Estimator> myEstimators;
    Splitter mySplitter;

    CountedConstPtrOrConstPtr<PointPredicate> myPointPredicate;
    CountedConstPtrOrConstPtr<KSpace> myKSpace;

    double myH;
    double myRadius;
  };
}

#include "ParallelIIEstimator.ih"
