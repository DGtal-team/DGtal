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
 * @file ATSolver2D.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2019/06/06
 *
 * Header file for module ATSolver2D.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ATSolver2D_RECURSES)
#error Recursive header files inclusion detected in ATSolver2D.h
#else // defined(ATSolver2D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ATSolver2D_RECURSES

#if !defined ATSolver2D_h
/** Prevents repeated inclusion of headers. */
#define ATSolver2D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <sstream>
#include <tuple>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConstAlias.h"
#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"
#include "DGtal/dec/DECHelpers.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ATSolver2D
  /**
  * Description of template class 'ATSolver2D' <p> \brief Aim:
  * This class solves Ambrosio-Tortorelli functional on a
  * two-dimensional digital space (a 2D grid or 2D digital surface)
  * for a piecewise smooth scalar/vector function \a u represented as
  * one/several 2-form(s) and a discontinuity function \a v
  * represented as a 0-form. The 2-form(s) \a u is a regularized
  * approximation of an input vector data \a g, while \a v represents
  * the set of discontinuities of \a u.  The norm chosen for \a u is
  * the \f$ l_2 \f$-norm.
  *
  * @tparam TKSpace any model of CCellularGridSpaceND, e.g KhalimskySpaceND
  *
  * @tparam TLinearAlgebra any back-end for performing linear algebra,
  * default is EigenLinearAlgebraBackend.
  *
  * \code
  * // Typical use (with appropriate definitions for types and variables).
  * typedef DiscreteExteriorCalculusFactory<EigenLinearAlgebraBackend> CalculusFactory;
  * const auto calculus = CalculusFactory::createFromNSCells<2>( surfels.begin(), surfels.end() );
  * ATSolver2D< KSpace > at_solver(calculus, 1);
  * at_solver.initInputVectorFieldU2( normals, surfels.cbegin(), surfels.cend() );
  * at_solver.setUp( alpha_at, lambda_at );
  * at_solver.solveGammaConvergence( 2.0, 0.5, 2.0 );
  * at_solver.getOutputVectorFieldU2( normals, surfels.cbegin(), surfels.cend() );
  * \endcode
  *
  * @see exampleSurfaceATNormals.cpp
  */
  template < typename TKSpace,
             typename TLinearAlgebra = EigenLinearAlgebraBackend >
  class ATSolver2D
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef TKSpace                                              KSpace;
    typedef TLinearAlgebra                                       LinearAlgebra;
    typedef ATSolver2D< KSpace, LinearAlgebra >                  Self;

    static const Dimension dimension = KSpace::dimension;

    /// Specifies how to merge the different values of 0-form v at cell vertices when
    /// outputting the 0-form v for a range of cells (either pointels, linels, surfels).
    enum CellOutputPolicy { Average, ///< compute average values at cell vertices
                            Minimum, ///< compute minimum value at cell vertices,
                            Maximum, ///< compute maximum value at cell vertices
    };
    typedef typename KSpace::Space                               Space;
    typedef typename Space::RealVector                           RealVector;
    typedef typename RealVector::Component                       Scalar;
    typedef typename KSpace::SCell                               SCell;
    typedef typename KSpace::Cell                                Cell;
    typedef typename KSpace::Surfel                              Surfel;
    typedef HyperRectDomain<Space>                               Domain;
    typedef DiscreteExteriorCalculus<2,dimension, LinearAlgebra> Calculus;
    typedef typename KSpace::template SurfelMap<double>::Type    SmallestEpsilonMap;
    typedef typename Calculus::Index                             Index;
    typedef typename Calculus::PrimalForm0                       PrimalForm0;
    typedef typename Calculus::PrimalForm1                       PrimalForm1;
    typedef typename Calculus::PrimalForm2                       PrimalForm2;
    typedef typename Calculus::PrimalIdentity0                   PrimalIdentity0;
    typedef typename Calculus::PrimalIdentity1                   PrimalIdentity1;
    typedef typename Calculus::PrimalIdentity2                   PrimalIdentity2;
    typedef typename Calculus::PrimalDerivative0                 PrimalDerivative0;
    typedef typename Calculus::PrimalDerivative1                 PrimalDerivative1;
    typedef typename Calculus::PrimalAntiderivative1             PrimalAntiderivative1;
    typedef typename Calculus::PrimalAntiderivative2             PrimalAntiderivative2;
    typedef typename Calculus::PrimalHodge0                      PrimalHodge0;
    typedef typename Calculus::PrimalHodge1                      PrimalHodge1;
    typedef typename Calculus::PrimalHodge2                      PrimalHodge2;
    typedef typename KSpace::template SurfelMap<Index>::Type     Surfel2IndexMap;

    // SparseLU is so much faster than SparseQR
    // SimplicialLLT is much faster than SparseLU
    // SimplicialLDLT is as fast as SimplicialLLT but more robust
    // typedef EigenLinearAlgebraBackend::SolverSparseQR LinearAlgebraSolver;
    // typedef EigenLinearAlgebraBackend::SolverSparseLU LinearAlgebraSolver;
    // typedef EigenLinearAlgebraBackend::SolverSimplicialLLT LinearAlgebraSolver;
    typedef EigenLinearAlgebraBackend::SolverSimplicialLDLT LinearAlgebraSolver;
    typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 2, PRIMAL, 2, PRIMAL> SolverU2;
    typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, PRIMAL, 0, PRIMAL> SolverV0;

  protected:
    /// A smart (or not) pointer to a calculus object.
    CountedConstPtrOrConstPtr< Calculus > ptrCalculus;
    /// the derivative operator for primal 0-forms
    PrimalDerivative0     primal_D0;
    /// the derivative operator for primal 1-forms
    PrimalDerivative1     primal_D1;
    /// The primal vertex to edge average operator
    PrimalDerivative0     M01;
    /// The primal edge to face average operator
    PrimalDerivative1     M12;
    /// The antiderivative of primal 2-forms
    PrimalAntiderivative2 primal_AD2;
    /// The alpha-weighted identity operator for primal 2-forms (stored for performance)
    PrimalIdentity2       alpha_Id2;
    /// The 1/(4epsilon)-weighted identity operator for primal 0-forms (stored for performance)
    PrimalIdentity0       l_1_over_4e_Id0;
    /// The N-array of input primal 2-forms g
    std::vector<PrimalForm2> g2;
    /// The alpha-weighted N-array of input primal 2-forms g
    std::vector<PrimalForm2> alpha_g2;
    /// The N-array of regularized primal 2-forms u
    std::vector<PrimalForm2> u2;
    /// The primal 0-form v representing the set of discontinuities S
    /// (more precisely \f$ 1 - \chi_S \f$)
    PrimalForm0           v0;
    /// The primal 0-form v at the previous iteration
    PrimalForm0           former_v0;
    /// The primal 0-form lambda/(4epsilon) (stored for performance)
    PrimalForm0           l_1_over_4e;

  public:
    // The map Surfel -> Index that gives the index of the surfel in 2-forms.
    Surfel2IndexMap       surfel2idx;
    /// The map Surfel -> double telling the smallest epsilon for
    /// which the surfel was a discontinuity.
    SmallestEpsilonMap    smallest_epsilon_map;
    /// The global coefficient alpha giving the smoothness of the
    /// reconstruction (the smaller, the smoother)
    double                alpha;
    /// The global coefficient lambda giving the length of
    /// discontinuities (the smaller, the more discontinuities)
    double                lambda;
    /// The global coefficient epsilon giving the width of the
    /// discontinuities (the smaller, the thinner)
    double                epsilon;
    /// Indicates whether to normalize U (unit norm) at each iteration or not.
    bool                  normalize_u2;
    /// Tells the verbose level.
    int                   verbose;

    // ----------------------- Standard services ------------------------------
    /// @name Standard services
    /// @{

    /// Prepare an AT-solver from a valid calculus.
    ///
    /// @param aCalculus any valid calculus
    /// @param aVerbose tells how the solver displays computing information: 0 none, 1 more, 2 even more...
    /// @see \ref DiscreteExteriorCalculusFactory for creating calculus objects.
    ATSolver2D( ConstAlias< Calculus > aCalculus, int aVerbose = 0 )
      : ptrCalculus( aCalculus ),
        primal_D0( *ptrCalculus ), primal_D1( *ptrCalculus ),
        M01( *ptrCalculus ), M12( *ptrCalculus ), primal_AD2( *ptrCalculus ),
        alpha_Id2( *ptrCalculus ), l_1_over_4e_Id0( *ptrCalculus ),
        g2(), alpha_g2(), u2(), v0( *ptrCalculus ), former_v0( *ptrCalculus ),
        l_1_over_4e( *ptrCalculus ), verbose( aVerbose )
    {
      if ( verbose >= 2 )
	trace.info() << "[ATSolver::ATSolver] " << *ptrCalculus << std::endl;
      initOperators();
      const auto size2 = ptrCalculus->kFormLength( 2, PRIMAL );
      for ( Index index = 0; index < size2; ++index) {
	const auto& calculus_cell = ptrCalculus->getSCell( 2, PRIMAL, index );
	surfel2idx[ calculus_cell ] = index;
      }
    }

    /**
     * Default constructor.
     */
    ATSolver2D() = delete;

    /**
     * Destructor.
     */
    ~ATSolver2D() = default;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    ATSolver2D ( const ATSolver2D & other ) = default;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    ATSolver2D ( ATSolver2D && other ) = default;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    ATSolver2D & operator= ( const ATSolver2D & other ) = default;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    ATSolver2D & operator= ( ATSolver2D && other ) = default;

    /// @param order the dimension of cells (0,1,2)
    /// @return the number of cells with dimension \a order
    Index size( const int order ) const
    {
      return ptrCalculus->kFormLength(order, PRIMAL);
    }

    /// @}


    // ----------------------- Initialization services ------------------------------
  public:
    /// @name Initialization services
    /// @{

    /// Given a range of surfels [itB,itE) and an input vector field,
    /// initializes the AT 2-forms u and g. The 0-form v is itself
    /// initialized to 1 everywhere.
    ///
    /// @tparam VectorFieldInput the type of vector field for input values (RandomAccess container)
    /// @tparam SurfelRangeConstIterator the type of iterator for traversing a range of surfels
    ///
    /// @param[in] input the input vector field (a vector of vector values)
    ///
    /// @param[in] itB the start of the range of surfels.
    /// @param[in] itE past the end of the range of surfels.
    ///
    /// @param[in] normalize when 'true', the input is supposed to be
    /// a unit vector field and the solver will output a unit
    /// regularized vector field at the end of each minimization step.
    template <typename VectorFieldInput, typename SurfelRangeConstIterator>
    void
    initInputVectorFieldU2( const VectorFieldInput& input,
                            SurfelRangeConstIterator itB, SurfelRangeConstIterator itE,
                            bool normalize = false )
    {
      if ( verbose >= 1 )
        trace.beginBlock( "[ATSolver2D::initInputVectorFieldU2] Initializing input data" );
      ASSERT( ! input.empty() );
      const Dimension N = input[ 0 ].size();
      if ( verbose >= 2 ) trace.info() << "input g as " << N << " 2-forms." << std::endl;
      g2       = std::vector<PrimalForm2>( N, PrimalForm2( *ptrCalculus ) );
      alpha_g2 = std::vector<PrimalForm2>( N, PrimalForm2( *ptrCalculus ) );
      Index i = 0;
      for ( auto it = itB; it != itE; ++it, ++i )
	{
	  Index idx = surfel2idx[ *it ];
	  for ( Dimension k = 0; k < N; ++k )
	    g2[ k ].myContainer( idx ) = input[ i ][ k ];
	}
      // u = g at the beginning
      if ( verbose >= 2 )
	trace.info() << "Unknown u[:] = g[:] at beginning." << std::endl;
       u2 = g2;
      // v = 1 at the beginning
      if ( verbose >= 2 ) trace.info() << "Unknown v = 1" << std::endl;
      v0 = PrimalForm0::ones(*ptrCalculus);
      if ( verbose >= 1 ) trace.endBlock();
      normalize_u2 = normalize;
    }

    /// Given a range of surfels [itB,itE) and an input scalar field,
    /// initializes the AT 2-forms u and g. The 0-form v is itself
    /// initialized to 1 everywhere.
    ///
    /// @tparam ScalarFieldInput the type of scalar field for input values (RandomAccess container)
    /// @tparam SurfelRangeConstIterator the type of iterator for traversing a range of surfels
    ///
    /// @param[in] input the input scalar field (a vector of scalar values)
    ///
    /// @param itB the start of the range of surfels.
    /// @param itE past the end of the range of surfels.
    template <typename ScalarFieldInput, typename SurfelRangeConstIterator>
    void
    initInputScalarFieldU2( const ScalarFieldInput& input,
                            SurfelRangeConstIterator itB, SurfelRangeConstIterator itE )
    {
      if ( verbose >= 1 )
        trace.beginBlock( "[ATSolver2D::initInputVectorFieldU2] Initializing input data" );
      ASSERT( ! input.empty() );
      if ( verbose >= 2 ) trace.info() << "input g as one 2-form." << std::endl;
      g2       = std::vector<PrimalForm2>( 1, PrimalForm2( *ptrCalculus ) );
      alpha_g2 = std::vector<PrimalForm2>( 1, PrimalForm2( *ptrCalculus ) );
      Index i = 0;
      for ( auto it = itB; it != itE; ++it, ++i )
	{
	  Index idx = surfel2idx[ *it ];
          g2[ 0 ].myContainer( idx ) = input[ i ];
	}
      // u = g at the beginning
      if ( verbose >= 2 )
	trace.info() << "Unknown u[:] = g[:] at beginning." << std::endl;
       u2 = g2;
      // v = 1 at the beginning
      if ( verbose >= 2 ) trace.info() << "Unknown v = 1" << std::endl;
      v0 = PrimalForm0::ones(*ptrCalculus);
      if ( verbose >= 1 ) trace.endBlock();
      normalize_u2 = false;
    }


    /// Given a map Surfel -> ScalarVector, initializes forms g, u and
    /// v of the AT solver. Note that there are as many 2-forms u/g as the
    /// number of dimensions of ScalarVector.
    ///
    /// @param input any map Surfel -> ScalarVector
    ///
    /// @param normalize when 'true', the input is supposed to be a
    /// unit vector field and the solver will output a unit
    /// regularized vector field at the end of each minimization step.
    ///
    /// @return the number of cells that were initialized.
    /// @tparam ScalarVector any type representing a vector/array of scalars (float, double)
    template <typename ScalarVector>
    Index
    initInputVectorFieldU2( const std::map<Surfel,ScalarVector>& input,
                            bool normalize = false )
    {
      if ( verbose >= 1 ) trace.beginBlock( "[ATSolver2D::initVectorInput] Initializing input data" );
      if ( verbose >= 2 ) trace.info() << "discontinuity 0-form v = 1." << std::endl;
      const Dimension N = ScalarVector().size();
      if ( verbose >= 2 ) trace.info() << "input g as " << N << " 2-forms." << std::endl;
      g2       = std::vector<PrimalForm2>( N, PrimalForm2( *ptrCalculus ) );
      alpha_g2 = std::vector<PrimalForm2>( N, PrimalForm2( *ptrCalculus ) );
      const ScalarVector zero;
      Index              nbok = 0;
      for ( Index index = 0; index < size(2); index++)
        {
          const SCell& cell = g2[ 0 ].getSCell( index );
          const auto   it   = input.find( cell );
          const auto   n    = ( it != input.end() ) ? it->second : zero;
          nbok             += ( it != input.end() ) ? 1 : 0;
          for ( Dimension k = 0; k < N; ++k )
            g2[ k ].myContainer( index ) = n[ k ];
        }
      // u = g at the beginning
      if ( verbose >= 2 )
	trace.info() << "Unknown u[:] = g[:] at beginning." << std::endl;
      u2 = g2;
      // v = 1 at the beginning
      if ( verbose >= 2 ) trace.info() << "Unknown v = 1" << std::endl;
      v0 = PrimalForm0::ones(*ptrCalculus);
      if ( verbose >= 1 ) trace.endBlock();
      normalize_u2 = normalize;
      return nbok;
    }

    /// Given a map Surfel -> Scalar, initializes forms g, u and
    /// v of the AT solver. Note that there is only one 2-form u/g.
    ///
    /// @param input any map Surfel -> Scalar
    /// @return the number of cells that were initialized.
    /// @tparam Scalar any type representing a scalar (float, double)
    template <typename Scalar>
    Index
    initInputScalarFieldU2( const std::map<Surfel,Scalar>& input )
    {
      if ( verbose >= 1 ) trace.beginBlock( "[ATSolver2D::initScalarInput] Initializing input data" );
      if ( verbose >= 2 ) trace.info() << "discontinuity 0-form v = 1." << std::endl;
      v0 = PrimalForm0::ones(*ptrCalculus);
      if ( verbose >= 2 ) trace.info() << "input g as one 2-form." << std::endl;
      g2       = std::vector<PrimalForm2>( 1, PrimalForm2( *ptrCalculus ) );
      alpha_g2 = std::vector<PrimalForm2>( 1, PrimalForm2( *ptrCalculus ) );
      const Scalar zero;
      Index nbok = 0;
      for ( Index index = 0; index < size(2); index++)
        {
          const SCell& cell = g2[ 0 ].getSCell( index );
          const auto   it   = input.find( cell );
          const auto   n    = ( it != input.end() ) ? *it : zero;
          nbok             += ( it != input.end() ) ? 1 : 0;
          g2[ 0 ].myContainer( index ) = n;
        }
      // u = g at the beginning
      if ( verbose >= 2 ) trace.info() << "Unknown u[:] = g[:] at beginning." << std::endl;
      u2 = g2;
      // v = 1 at the beginning
      if ( verbose >= 2 ) trace.info() << "Unknown v = 1" << std::endl;
      v0 = PrimalForm0::ones(*ptrCalculus);
      if ( verbose >= 1 ) trace.endBlock();
      normalize_u2 = false;
      return nbok;
    }

    /// Initializes the alpha and lambda parameters of AT.
    /// @param a the global alpha parameter
    /// @param l the global lambda parameter
    /// @note all 2-cells have the same weight for the data term.
    void setUp( double a, double l )
    {
      const Dimension N = (Dimension)g2.size();
      alpha     = a;
      lambda    = l;
      alpha_Id2 = alpha * ptrCalculus->template identity<2, PRIMAL>();
      for ( Dimension k = 0; k < N; ++k )
        alpha_g2[ k ] = alpha * g2[ k ];
    }

    /// Initializes the alpha and lambda parameters of AT, with
    /// weights on the 2-cells for the data terms.
    ///
    /// @param a the global alpha parameter
    /// @param l the global lambda parameter
    /// @param weights the map Surfel -> Scalar that gives the weight of each 2-cell in the data terms.
    ///
    /// @note Useful for inpainting applications or for adaptive
    /// piecewise smooth reconstruction.
    void setUp( double a, double l, const std::map<Surfel,Scalar>& weights )
    {
      const Dimension N = g2.size();
      alpha  = a;
      lambda = l;
      PrimalForm2 w_form( *ptrCalculus );
      if ( verbose >= 2 )
        trace.info() << "Using variable weights for fitting (alpha term)" << std::endl;
      for ( Dimension k = 0; k < N; ++k )
        alpha_g2[ k ] = alpha * g2[ k ];
      for ( Index index = 0; index < size( 2 ); index++)
        {
          const SCell&           cell = g2[ 0 ].getSCell( index );
          const Scalar&             w = weights[ cell ];
          w_form.myContainer( index ) = w;
          for ( Dimension k = 0; k < N; ++k )
            alpha_g2[ k ].myContainer( index ) *= w;
        }
      alpha_Id2 = alpha * diagonal( w_form );
    }

    /// Initializes the alpha and lambda parameters of AT, with
    /// weights on the 2-cells for the data terms.
    ///
    /// @tparam AlphaWeights the type of RandomAccess container for alpha weight values
    /// @tparam SurfelRangeConstIterator the type of iterator for traversing a range of surfels
    ///
    /// @param[in] a the global alpha parameter
    /// @param[in] l the global lambda parameter
    /// @param[in] weights the vector of alpha weights for each surfel of the range [itB,itE)
    /// @param[in] itB the start of the range of surfels.
    /// @param[in] itE past the end of the range of surfels.
    ///
    /// @note Useful for inpainting applications or for adaptive
    /// piecewise smooth reconstruction.
    template <typename AlphaWeights, typename SurfelRangeConstIterator>
    void setUp( double a, double l,
                const AlphaWeights& weights,
                SurfelRangeConstIterator itB, SurfelRangeConstIterator itE )
    {
      const Dimension N = g2.size();
      alpha  = a;
      lambda = l;
      PrimalForm2 w_form( *ptrCalculus );
      if ( verbose >= 2 )
        trace.info() << "Using variable weights for fitting (alpha term)" << std::endl;
      for ( Dimension k = 0; k < N; ++k )
        alpha_g2[ k ] = alpha * g2[ k ];
      Index i = 0;
      for ( auto it = itB; it != itE; ++it, ++i )
	{
	  const Index idx = surfel2idx[ *it ];
          const Scalar  w = weights[ i ];
          w_form.myContainer( idx ) = w;
	  for ( Dimension k = 0; k < N; ++k )
	    alpha_g2[ k ].myContainer( idx ) *= w;
	}
      alpha_Id2 = alpha * diagonal( w_form );
    }

    /// Initializes the epsilon parameter of AT and precomputes the assaociated forms and operators.
    /// @param e the epsilon parameter in AT
    void setEpsilon( double e )
    {
      epsilon         = e;
      l_1_over_4e     = (lambda/4./epsilon)*PrimalForm0::ones(*ptrCalculus);
      l_1_over_4e_Id0 = (lambda/4./epsilon)*ptrCalculus->template identity<0, PRIMAL>();
    }

    /// @}

    // ----------------------- Optimization services ------------------------------
  public:
    /// @name Optimization services
    /// @{

    /// Solves one step of the alternate minimization of AT. Solves
    /// for u then for v.
    ///
    /// @return true if everything went fine, false if there was a
    /// problem in the optimization.
    ///
    /// @note Use \ref diffV0 to check if you are close to a critical point of AT.
    bool solveOneAlternateStep()
    {
      bool solve_ok = true;
      if ( verbose >= 1 ) trace.beginBlock("Solving for u as a 2-form");
      PrimalForm1 v1_squared = M01*v0;
      v1_squared.myContainer.array() = v1_squared.myContainer.array().square();
      const PrimalIdentity2 ope_u2 = alpha_Id2
        + primal_AD2.transpose() * dec_helper::diagonal( v1_squared ) * primal_AD2;

      if ( verbose >= 2 ) trace.info() << "Prefactoring matrix U associated to u" << std::endl;
      SolverU2 solver_u2;
      solver_u2.compute( ope_u2 );
      for ( Dimension d = 0; d < u2.size(); ++d )
        {
          if ( verbose >= 2 ) trace.info() << "Solving U u[" << d << "] = a g[" << d << "]" << std::endl;
          u2[ d ] = solver_u2.solve( alpha_g2[ d ] );
          if ( verbose >= 2 ) trace.info() << "  => " << ( solver_u2.isValid() ? "OK" : "ERROR" )
                                           << " " << solver_u2.myLinearAlgebraSolver.info() << std::endl;
          solve_ok = solve_ok && solver_u2.isValid();
        }
      if ( normalize_u2 ) normalizeU2();
      if ( verbose >= 1 ) trace.endBlock();
      if ( verbose >= 1 ) trace.beginBlock("Solving for v");
      former_v0 = v0;
      PrimalForm1 squared_norm_d_u2 = PrimalForm1::zeros(*ptrCalculus);
      for ( Dimension d = 0; d < u2.size(); ++d )
        squared_norm_d_u2.myContainer.array() += (primal_AD2 * u2[ d ] ).myContainer.array().square();
      trace.info() << "build metric u2" << std::endl;
      const PrimalIdentity0 ope_v0 = l_1_over_4e_Id0
        + (lambda * epsilon) * primal_D0.transpose() * primal_D0
	+ M01.transpose() * dec_helper::diagonal( squared_norm_d_u2 ) * M01;

      if ( verbose >= 2 ) trace.info() << "Prefactoring matrix V associated to v" << std::endl;
      SolverV0 solver_v0;
      solver_v0.compute( ope_v0 );
      if ( verbose >= 2 ) trace.info() << "Solving V v = l/4e * 1" << std::endl;
      v0 = solver_v0.solve( l_1_over_4e );
      if ( verbose >= 2 ) trace.info() << "  => " << ( solver_v0.isValid() ? "OK" : "ERROR" )
                                       << " " << solver_v0.myLinearAlgebraSolver.info() << std::endl;
      solve_ok = solve_ok && solver_v0.isValid();
      if ( verbose >= 1 ) trace.endBlock();
      return solve_ok;
    }

    /// Solves the alternate minimization of AT for a given \a eps. Solves
    /// for u then for v till convergence.
    ///
    /// @param eps the epsilon parameter at which AT is solved.
    ///
    /// @param n_oo_max the alternate minimization will stop when the
    /// loo-norm of \f$ v^{k+1} - v^k \f$ is below this bound.
    ///
    /// @param iter_max the alternate minimization will stop when the
    /// number of minimization steps exceeds \a iter_max.
    ///
    /// @return true if everything went fine, false if there was a
    /// problem in the optimization.
    ///
    /// @note Use \ref diffV0 to check if you are close to a critical point of AT.
    bool solveForEpsilon( double eps,
			  double n_oo_max = 1e-4,
			  unsigned int iter_max = 10 )
    {
      (void)n_oo_max;//parameter not used

      bool ok = true;
      if ( verbose >= 1 ) {
	std::ostringstream sstr;
	sstr << "******* Solving AT for epsilon = " << eps << " **********";
	trace.beginBlock( sstr.str() );
      }
      setEpsilon( eps );
      for ( unsigned int i = 0; i < iter_max; ++i )
        {
	  if ( verbose >= 1 )
	    trace.info() << "---------- Iteration "
			 << i << "/" << iter_max << " ---------------" << std::endl;
	  solveOneAlternateStep( );
	  auto diffs_v = diffV0();
	  if ( verbose >= 1 ) {
	    trace.info() << "Variation |v^k+1 - v^k|_oo = " << std::get<0>( diffs_v )
			 << std::endl;
	    if ( verbose >= 2 ) {
	      trace.info() << "Variation |v^k+1 - v^k|_2  = " << std::get<1>( diffs_v )
			   << std::endl;
	      trace.info() << "Variation |v^k+1 - v^k|_1  = " << std::get<2>( diffs_v )
			   << std::endl;
	    }
	  }
	  if ( std::get<0>( diffs_v ) < 1e-4 ) break;
        }
      if ( verbose >= 1 ) trace.endBlock();
      return ok;
    }

    /// Solves AT by progressively decreasing epsilon from \a eps1
    /// to \a eps2. AT is solved with solveForEpsilon at each epsilon.
    ///
    /// @param eps1 the first epsilon parameter at which AT is solved.
    /// @param eps2 the last epsilon parameter at which AT is solved.
    /// @param epsr the ratio (>1) used to decrease progressively epsilon.
    /// @param compute_smallest_epsilon_map when 'true' determines for each surfel the smallest epsilon for which it is a discontinuity.
    ///
    /// @param n_oo_max the alternate minimization will stop when the
    /// loo-norm of \f$ v^{k+1} - v^k \f$ is below this bound.
    ///
    /// @param iter_max the alternate minimization will stop when the
    /// number of minimization steps exceeds \a iter_max.
    ///
    /// @return true if everything went fine, false if there was a
    /// problem in the optimization.
    bool solveGammaConvergence( double eps1 = 2.0,
				double eps2 = 0.25,
				double epsr = 2.0,
				bool compute_smallest_epsilon_map = false,
				double n_oo_max = 1e-4,
				unsigned int iter_max = 10 )
    {
      bool ok = true;
      if ( epsr <= 1.0 ) epsr = 2.0;
      if ( verbose >= 1 )
	trace.beginBlock( "#### Solve AT by Gamma-convergence ##########" );
      if ( compute_smallest_epsilon_map ) smallest_epsilon_map.clear();
      for ( double eps = eps1; eps >= eps2; eps /= epsr )
	{
	  solveForEpsilon( eps, n_oo_max, iter_max );
	  if ( compute_smallest_epsilon_map )
	    updateSmallestEpsilonMap( 0.5 );
	}
      if ( verbose >= 1 )
	trace.endBlock();
      return ok;
    }

    /// Forces the normalization of the vector u, meaning for all
    /// index i, \f$ \sum_{k=0}^{K-1} u[k][i]^2 = 1 \f$. Can be useful
    /// in some applications where you are looking for unitary vector
    /// field.
    void normalizeU2()
    {
      for ( Index index = 0; index < size( 2 ); index++)
        {
          double n2 = 0.0;
          for ( unsigned int d = 0; d < u2.size(); ++d )
            n2 += u2[ d ].myContainer( index ) * u2[ d ].myContainer( index );
          double norm = sqrt( n2 );
          if (norm == 0.0) continue;
          for ( unsigned int d = 0; d < u2.size(); ++d )
            u2[ d ].myContainer( index ) /= norm;
        }
    }

    /// Computes the norms loo, l2, l1 of (v - former_v), i.e. the
    /// evolution of discontinuity function v.
    ///
    /// @return a tuple (n_infty,n_2,n_1) giving  the loo/l2/l1-norm of (v - former_v)
    std::tuple<double,double,double> diffV0() const
    {
      PrimalForm0 delta = v0 - former_v0;
      delta.myContainer = delta.myContainer.cwiseAbs();
      const double n_oo = delta.myContainer.maxCoeff();
      const double n_2  = std::sqrt(delta.myContainer.squaredNorm()/delta.myContainer.size());
      const double n_1  = delta.myContainer.mean();
      return std::make_tuple( n_oo, n_2, n_1 );
    }

    /// @}


    // ----------------------- Access services ------------------------------
  public:
    /// @name Access services
    /// @{

    /// Debug method for checking if v is a scalar field between 0 and 1.
    ///
    /// @return the tuple (min(v), average(v), max(v))
    std::tuple<double,double,double> checkV0() const
    {
      const double m1 = v0.myContainer.minCoeff();
      const double m2 = v0.myContainer.maxCoeff();
      const double ma = v0.myContainer.mean();
      if ( verbose >= 1 )
        trace.info() << "0-form v (should be in [0,1]): min=" << m1 << " avg=" << ma << " max=" << m2 << std::endl;
      return std::make_tuple( m1, m2, ma );
    }


    /// @return the discontinuity function v as a primal 0-form.
    PrimalForm0 getV0() const
    {
      return v0;
    }

    /// @return the discontinuity function v as a primal 1-form.
    PrimalForm1 getV1() const
    {
      return M01*v0;
    }

    /// @return the discontinuity function u as a primal 2-form.
    PrimalForm2 getV2() const
    {
      return M12*M01*v0;
    }

    /// @param k an integer such that `0 <= k < u2.size()`
    /// @return the k-th piecewise smooth function u as a primal 2-form.
    PrimalForm2 getU2( Dimension k ) const
    {
      return u2[ k ];
    }

    /// Given a range of surfels [itB,itE), returns in \a output the
    /// regularized vector field u.
    ///
    /// @tparam VectorFieldOutput the type of vector field for output values (RandomAccess container)
    /// @tparam SurfelRangeConstIterator the type of iterator for traversing a range of surfels
    ///
    /// @param[out] output the vector of output values (a scalar or
    /// vector field depending on input).
    ///
    /// @param itB the start of the range of surfels.
    /// @param itE past the end of the range of surfels.
    template <typename VectorFieldOutput, typename SurfelRangeConstIterator>
    void
    getOutputVectorFieldU2( VectorFieldOutput& output,
                            SurfelRangeConstIterator itB, SurfelRangeConstIterator itE )
    {
      const Dimension N = u2.size();
      Index i = 0;
      for ( auto it = itB; it != itE; ++it, ++i )
	{
	  Index idx = surfel2idx[ *it ];
          ASSERT( output[ i ].size() >= N );
	  for ( Dimension k = 0; k < N; ++k )
	    output[ i ][ k ] = u2[ k ].myContainer( idx );
	}
    }

    /// Given a range of surfels [itB,itE), returns in \a output the
    /// regularized scalar field u.
    ///
    /// @tparam ScalarFieldOutput the type of scalar field for output values (RandomAccess container)
    /// @tparam SurfelRangeConstIterator the type of iterator for traversing a range of surfels
    ///
    /// @param[out] output the vector of output values (a scalar field), which should be of size `length(itB,itE)`
    ///
    /// @param itB the start of the range of surfels.
    /// @param itE past the end of the range of surfels.
    template <typename ScalarFieldOutput, typename SurfelRangeConstIterator>
    void
    getOutputScalarFieldU2( ScalarFieldOutput& output,
                            SurfelRangeConstIterator itB, SurfelRangeConstIterator itE )
    {
      ASSERT( u2.size() == 1 && "[ATSolver2D::getOutputScalarFieldU2] "
              "You try to output a scalar field from a vector field." );
      Index i = 0;
      for ( auto it = itB; it != itE; ++it, ++i )
	{
	  Index idx = surfel2idx[ *it ];
          output[ i ] = u2[ 0 ].myContainer( idx );
	}
    }

    /// Given a range of pointels, linels or 2-cells [itB,itE), returns in \a output the
    /// feature vector \a v (the average of \a v for linels/surfels).
    ///
    /// @tparam ScalarFieldOutput the type of scalar field for output values (RandomAccess container)
    /// @tparam CellRangeConstIterator the type of iterator for traversing a range of cells
    ///
    /// @param[out] output the vector of output scalar values (a scalar field), which should be of size `length(itB,itE)`
    ///
    /// @param[in] itB the start of the range of cells.
    /// @param[in] itE past the end of the range of cells.
    /// @param[in] policy the chosen policy for outputting v values for a given cell.
    template <typename ScalarFieldOutput, typename CellRangeConstIterator>
    void
    getOutputScalarFieldV0( ScalarFieldOutput& output,
                            CellRangeConstIterator itB, CellRangeConstIterator itE,
                            CellOutputPolicy policy = CellOutputPolicy::Average )
    {
      const KSpace& K = ptrCalculus->myKSpace;
      const Dimension k = K.uDim( *itB );
      ASSERT( k <= 2 );
      Index i = 0;
      if ( k == 0 )
        {
          for ( auto it = itB; it != itE; ++it, ++i )
            {
              const Cell pointel = *it;
              const Index    idx = ptrCalculus->getCellIndex( pointel );
              output[ i ] = v0.myContainer( idx );
            }
        }
      else if ( k == 1 )
        {
          for ( auto it = itB; it != itE; ++it, ++i )
            {
              const Cell  linel = *it;
              const Dimension d = * K.uDirs( linel );
              const Cell     p0 = K.uIncident( linel, d, false );
              const Cell     p1 = K.uIncident( linel, d, true  );
              const Index  idx0 = ptrCalculus->getCellIndex( p0 );
              const Index  idx1 = ptrCalculus->getCellIndex( p1 );
              switch (policy) {
              case CellOutputPolicy::Average: output[ i ] = 0.5 * ( v0.myContainer( idx0 ) + v0.myContainer( idx1 ) );
                break;
              case CellOutputPolicy::Minimum: output[ i ] = std::min( v0.myContainer( idx0 ), v0.myContainer( idx1 ) );
                break;
              case CellOutputPolicy::Maximum: output[ i ] = std::max( v0.myContainer( idx0 ), v0.myContainer( idx1 ) );
                break;
              }
            }
        }
      else if ( k == 2 )
        {
          for ( auto it = itB; it != itE; ++it, ++i )
            {
              const Cell   face = *it;
              const Dimension d = * K.uDirs( face );
              const Cell     l0 = K.uIncident( face, d, false );
              const Cell     l1 = K.uIncident( face, d, true  );
              const Dimension j = * K.uDirs( l0 );
              const Cell    p00 = K.uIncident( l0, j, false );
              const Cell    p01 = K.uIncident( l0, j, true  );
              const Cell    p10 = K.uIncident( l1, j, false );
              const Cell    p11 = K.uIncident( l1, j, true  );
              const Index idx00 = ptrCalculus->getCellIndex( p00 );
              const Index idx01 = ptrCalculus->getCellIndex( p01 );
              const Index idx10 = ptrCalculus->getCellIndex( p10 );
              const Index idx11 = ptrCalculus->getCellIndex( p11 );
              switch (policy) {
              case CellOutputPolicy::Average:
                output[ i ] = 0.25 * ( v0.myContainer( idx00 ) + v0.myContainer( idx01 )
                                       + v0.myContainer( idx10 ) + v0.myContainer( idx11 ) );
                break;
              case CellOutputPolicy::Minimum:
                output[ i ] = std::min( std::min( v0.myContainer( idx00 ), v0.myContainer( idx01 ) ),
                                        std::min( v0.myContainer( idx10 ), v0.myContainer( idx11 ) ) );
                break;
              case CellOutputPolicy::Maximum:
                output[ i ] = std::max( std::max( v0.myContainer( idx00 ), v0.myContainer( idx01 ) ),
                                        std::max( v0.myContainer( idx10 ), v0.myContainer( idx11 ) ) );
                break;
              }
            }
        }
    }

    /// Computes the map that stores for each surfel the smallest
    /// epsilon for which the surfel was in the discontinuity zone
    /// (more precisely, the surfel has at least two vertices that
    /// belongs to the set of discontinuity).
    ///
    /// @param[in] threshold the threshold for discontinuity function
    /// v (below u is discontinuous, above u is continuous)
    void updateSmallestEpsilonMap( const double threshold = .5 )
    {
      const KSpace& K = ptrCalculus->myKSpace;
      for ( const SCell& surfel : ptrCalculus->template getIndexedSCells<2, PRIMAL>() )
        {
          const Cell face            = K.unsigns( surfel );
          const Dimension    k1      = * K.uDirs( face );
          const Cell   l0      = K.uIncident( face, k1, false );
          const Cell   l1      = K.uIncident( face, k1, true );
          const Dimension    k2      = * K.uDirs( l0 );
          const Cell   ll0     = K.uIncident( face, k2, false );
          const Cell   ll1     = K.uIncident( face, k2, true );
          const Cell   p00     = K.uIncident( l0, k2, false );
          const Cell   p01     = K.uIncident( l0, k2, true );
          const Cell   p10     = K.uIncident( l1, k2, false );
          const Cell   p11     = K.uIncident( l1, k2, true );

          std::vector<double>  features( 4 );
          features[ 0 ] = v0.myContainer( ptrCalculus->getCellIndex( p00 ) );
          features[ 1 ] = v0.myContainer( ptrCalculus->getCellIndex( p01 ) );
          features[ 2 ] = v0.myContainer( ptrCalculus->getCellIndex( p10 ) );
          features[ 3 ] = v0.myContainer( ptrCalculus->getCellIndex( p11 ) );
          std::sort( features.begin(), features.end() );

          if ( features[ 1 ] <= threshold )
            {
              auto it = smallest_epsilon_map.find( surfel );
              if ( it != smallest_epsilon_map.end() )
                it->second = std::min( epsilon, it->second );
              else smallest_epsilon_map[ surfel ] = epsilon;
            }
        }
    }

    /// @}


    // ----------------------- Interface --------------------------------------
  public:
    /// @name Interface services
    /// @{

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const
    {
      auto cv = checkV0();
      out << "[ATSolver2D] v is between min/avg/max:"
          << std::get<0>(cv) << "/"
          << std::get<1>(cv) << "/"
          << std::get<2>(cv) << std::endl;
    }

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const
    {
      return true;
    }

    /// @}

    // ------------------------- Hidden services ------------------------------
  protected:

    /// @name Hidden services
    /// @{

    /// Initializes the operators
    void initOperators()
    {
      if ( verbose >= 1 ) trace.beginBlock( "[ATSolver2D::initOperators] Solver initialization" );
      if ( verbose >= 2 ) trace.info() << "derivative of primal 0-forms: primal_D0" << std::endl;
      primal_D0 = ptrCalculus->template derivative<0,PRIMAL>();
      if ( verbose >= 2 ) trace.info() << "derivative of primal 1-forms: primal_D1" << std::endl;
      primal_D1 = ptrCalculus->template derivative<1,PRIMAL>();
      if ( verbose >= 2 ) trace.info() << "antiderivative of primal 2-forms: primal_AD2" << std::endl;
      primal_AD2 = ptrCalculus->template antiderivative<2,PRIMAL>();
      if ( verbose >= 2 ) trace.info() << "vertex to edge average operator: M01" << std::endl;
      M01       = primal_D0;
      M01.myContainer = .5 * M01.myContainer.cwiseAbs();
      if ( verbose >= 2 ) trace.info() << "edge to face average operator: M12" << std::endl;
      M12       = primal_D1;
      M12.myContainer = .25 * M12.myContainer.cwiseAbs();
      if ( verbose >= 1 ) trace.endBlock();
    }

    /// @}

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ATSolver2D


  /**
   * Overloads 'operator<<' for displaying objects of class 'ATSolver2D'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ATSolver2D' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const ATSolver2D<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ATSolver2D_h

#undef ATSolver2D_RECURSES
#endif // else defined(ATSolver2D_RECURSES)
