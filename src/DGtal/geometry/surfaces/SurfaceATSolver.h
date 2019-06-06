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
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2019/06/06
 *
 * Header file for module SurfaceATSolver.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SurfaceATSolver_RECURSES)
#error Recursive header files inclusion detected in SurfaceATSolver.h
#else // defined(SurfaceATSolver_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SurfaceATSolver_RECURSES

#if !defined SurfaceATSolver_h
/** Prevents repeated inclusion of headers. */
#define SurfaceATSolver_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class SurfaceATSolver
  /**
  * Description of template class 'SurfaceATSolver' <p> \brief Aim:
  * This class solves Ambrosio-Tortorelli functional on a digital
  * surface for \a u a (vector of) 2-form(s) and \a v a 0-form. \a u
  * is a regularized approximation of an input vector data \a g, while
  * \a v represents the set of discontinuities of \a u.
  * The norm chosen for \a u is the l2-norm.
  *
  * @tparam TKSpace any model of CCellularGridSpaceND, e.g KhalimskySpaceND
  * @tparam TLinearAlgebra any back-end for performing linear algebra, default is EigenLinearAlgebraBackend.
  *
  */
  template < typename TKSpace,
             typename TLinearAlgebra = EigenLinearAlgebraBackend >
  class SurfaceATSolver
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef TKSpace                                                    KSpace;
    typedef TLinearAlgebra                                             LinearAlgebra;
    typedef SurfaceATSolver< KSpace, N, LinearAlgebra >                Self;
    BOOST_STATIC_ASSERT(( N >= 1 ));
    
    typedef typename KSpace::Space                                     Space;
    typedef typename Space::RealVector                                 RealVector;
    typedef typename RealVector::Component                             Scalar;
    typedef typename KSpace::SCell                                     SCell;
    typedef typename KSpace::Cell                                      Cell;
    typedef typename KSpace::Surfel                                    Surfel;
    typedef HyperRectDomain<Space>                                     Domain;
    typedef DiscreteExteriorCalculus<2,3, LinearAlgebra>               Calculus;
    typedef DiscreteExteriorCalculusFactory<LinearAlgebra>             CalculusFactory;
    typedef std::map<Cell, double>                                     SmallestEpsilonMap;
    typedef Calculus::Index                                            Index;
    typedef Calculus::PrimalForm0                                      PrimalForm0;
    typedef Calculus::PrimalForm1                                      PrimalForm1;
    typedef Calculus::PrimalForm2                                      PrimalForm2;
    typedef Calculus::PrimalIdentity0                                  PrimalIdentity0;
    typedef Calculus::PrimalIdentity1                                  PrimalIdentity1;
    typedef Calculus::PrimalIdentity2                                  PrimalIdentity2;
    typedef Calculus::PrimalDerivative0                                PrimalDerivative0;
    typedef Calculus::PrimalDerivative1                                PrimalDerivative1;
    typedef Calculus::PrimalAntiderivative1                            PrimalAntiderivative1;
    typedef Calculus::PrimalAntiderivative2                            PrimalAntiderivative2;
    typedef Calculus::PrimalHodge0                                     PrimalHodge0;
    typedef Calculus::PrimalHodge1                                     PrimalHodge1;
    typedef Calculus::PrimalHodge2                                     PrimalHodge2;

    // SparseLU is so much faster than SparseQR
    // SimplicialLLT is much faster than SparseLU
    // SimplicialLDLT is as fast as SimplicialLLT but more robust
    // typedef EigenLinearAlgebraBackend::SolverSparseQR LinearAlgebraSolver;
    // typedef EigenLinearAlgebraBackend::SolverSparseLU LinearAlgebraSolver;
    // typedef EigenLinearAlgebraBackend::SolverSimplicialLLT LinearAlgebraSolver;
    typedef EigenLinearAlgebraBackend::SolverSimplicialLDLT LinearAlgebraSolver;
    typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 2, PRIMAL, 2, PRIMAL> SolverU2;
    typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, PRIMAL, 0, PRIMAL> SolverV0;

    /// A smart (or not) pointer to a calculus object.
    CountedConstPtrOrConstPtr< Calculus > calculus;
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
    /// The primal 0-form v representing the set of discontinuities S (more precisely \f$ 1 - \chi_S \f$)
    PrimalForm0           v0;
    /// The primal 0-form v at the previous iteration
    PrimalForm0           former_v0;
    /// The primal 0-form lambda/(4epsilon) (stored for performance)
    PrimalForm0           l_1_over_4e;
    /// The global coefficient alpha giving the smoothness of the reconstruction (the smaller, the smoother)
    double                alpha;
    /// The global coefficient lambda giving the length of discontinuities (the smaller, the more discontinuities)
    double                lambda;
    /// The global coefficient epsilon giving the width of the discontinuities (the smaller, the thinner)
    double                epsilon;
    /// Tells the verbose level.
    int                   verbose;
    
    /// Prepare an AT-solver from a valid calculus.
    /// @param aCalculus any valid calculus
    /// @param aVerbose tells how the solver displays computing information: 0 none, 1 more, 2 even more...
    /// @see CalculusFactory for creating calculus objects.
    ATSolver( ConstAlias< Calculus > aCalculus, int aVerbose = 0 )
      : calculus( aCalculus ),
        primal_D0( calculus ), primal_D1( calculus ),
        M01( calculus ), M12( calculus ), primal_AD2( calculus ),
        alpha_Id2( calculus ), l_1_over_4e_Id0( calculus ),
        g2(), alpha_g2(), u2(), v0( calculus ), former_v0( calculus ),
        l_1_over_4e( calculus ), verbose( aVerbose )
    {
      if ( verbose >= 2 ) trace.info() << "[ATSolver::ATSolver] " << calculus << std::endl;
      initOperators();
    }

    /// @param order the dimension of cells (0,1,2)
    /// @return the number of cells with dimension \a order
    Index size( const int order ) const
    {
      return calculus.kFormLength(order, PRIMAL);
    }

    /// Given a map Surfel -> ScalarVector, initializes forms g, u and
    /// v of the AT solver. Note that there are as many 2-forms u/g as the
    /// number of dimensions of ScalarVector.
    ///
    /// @param input any map Surfel -> ScalarVector
    /// @return the number of cells that were initialized.
    /// @tparam ScalarVector any type representing a vector/array of scalars (float, double)
    template <typename ScalarVector>
    Index
    initVectorInput( const std::map<Surfel,ScalarVector>& input )
    {
      if ( verbose >= 1 ) trace.beginBlock( "[SurfaceATSolver::initVectorInput] Initializing input data" );
      if ( verbose >= 2 ) trace.info() << "discontinuity 0-form v = 1." << std::endl;
      v0 = PrimalForm0::ones(calculus);
      const Dimension N = ScalarVector().size();
      if ( verbose >= 2 ) trace.info() << "input g as " << N << " 2-forms." << std::endl;
      g2       = std::vector<PrimalForm2>( N, PrimalForm2( *calculus ) );
      alpha_g2 = std::vector<PrimalForm2>( N, PrimalForm2( *calculus ) );
      const ScalarVector zero;
      Index              nbok = 0;
      for ( Index index = 0; index < size(2); index++)
        {
          const SCell& cell = g2[ 0 ].getSCell( index );
          const auto   it   = input.find( cell );  
          const auto   n    = ( it != input.end() ) ? *it : zero;
          nbok             += ( it != input.end() ) ? 1 : 0;
          for ( Dimension k = 0; k < N; ++k )
            g2[ k ].myContainer( index ) = n[ k ];
        }
      // u = g at the beginning
      if ( verbose >= 2 ) trace.info() << "Unknown u[:] = g[:] at beginning." << std::endl;
      u2 = g2;
      // v = 1 at the beginning
      if ( verbose >= 2 ) trace.info() << "Unknown v = 1" << std::endl;
      v0 = PrimalForm0::ones(calculus);
      if ( verbose >= 1 ) trace.endBlock();
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
    initScalarInput( const std::map<Surfel,Scalar>& input )
    {
      if ( verbose >= 1 ) trace.beginBlock( "[SurfaceATSolver::initScalarInput] Initializing input data" );
      if ( verbose >= 2 ) trace.info() << "discontinuity 0-form v = 1." << std::endl;
      v0 = PrimalForm0::ones(calculus);
      if ( verbose >= 2 ) trace.info() << "input g as one 2-form." << std::endl;
      g2       = std::vector<PrimalForm2>( 1, PrimalForm2( *calculus ) );
      alpha_g2 = std::vector<PrimalForm2>( 1, PrimalForm2( *calculus ) );
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
      v0 = PrimalForm0::ones(calculus);
      if ( verbose >= 1 ) trace.endBlock();
      return nbok;
    }

    /// Initializes the alpha and lambda parameters of AT.
    /// @param a the global alpha parameter
    /// @param l the global lambda parameter
    /// @note all 2-cells have the same weight for the data term.
    void setUp( double a, double l )
    {
      alpha     = a;
      lambda    = l;
      alpha_Id2 = alpha * calculus.identity<2, PRIMAL>();
      for ( Dimension k = 0; k < g2.size(); ++k )
        alpha_g2[ k ] = alpha * g2[ k ];
    }

    /// Initializes the alpha and lambda parameters of AT, with weights on the 2-cells for the data terms.
    /// @param a the global alpha parameter
    /// @param l the global lambda parameter
    /// @param weights the map Surfel -> Scalar that gives the weight of each 2-cell in the data terms.
    void setUp( double a, double l, const std::map<Surfel,Scalar>& weights )
    {
      alpha  = a;
      lambda = l;
      PrimalForm2 w_form( calculus );
      trace.info() << "Using variable weights for fitting (alpha term)" << std::endl;
      for ( Dimension k = 0; k < g2.size(); ++k )
        alpha_g2[ k ] = alpha * g2[ k ];
      for ( Index index = 0; index < nb(2); index++)
        {
          const SCell&           cell = g2[ 0 ].getSCell( index );
          const Scalar&             w = weights[ cell ];
          w_form.myContainer( index ) = w;
          for ( Dimension k = 0; k < g2.size(); ++k )
            alpha_g2[ k ].myContainer( index ) *= w;
        }
      alpha_Id2 = alpha * diagonal( w_form );
    }

    /// Initializes the epsilon parameter of AT and precomputes the assaociated forms and operators.
    /// @param e the epsilon parameter in AT
    void setEpsilon( double e )
    {
      epsilon     = e;
      l_1_over_4e = lambda/4/epsilon*PrimalForm0::ones(calculus);
      l_1_over_4e_Id0 = lambda/4/epsilon*calculus.identity<0, PRIMAL>();
    }


    /// Solves one step of the alternate minimization of AT.
    /// @param normalize_U2 when 'true', forces u to be a unit vector at the end of the minimization step.
    /// @note Use \ref diff_v0 to check if you are close to a critical point of AT.
    void solveU2V0( bool normalize_U2 = false )
    {
      if ( verbose >= 1 ) trace.beginBlock("Solving for u as a 2-form");
      PrimalForm1 v1_squared = M01*v0;
      v1_squared.myContainer.array() = v1_squared.myContainer.array().square();
      const PrimalIdentity2 ope_u2 = alpha_Id2 + primal_AD2.transpose()*dec_helper::diagonal(v1_squared)*primal_AD2;

      trace.info() << "Prefactoring matrix U" << std::endl;
      SolverU2 solver_u2;
      solver_u2.compute( ope_u2 );
      for ( unsigned int d = 0; d < 3; ++d )
        {
          trace.info() << "Solving U u[" << d << "] = a g[" << d << "]" << std::endl;
          u2[ d ] = solver_u2.solve( alpha_g2[ d ] );
          trace.info() << "  => " << ( solver_u2.isValid() ? "OK" : "ERROR" )
                       << " " << solver_u2.myLinearAlgebraSolver.info() << std::endl;
        }
      if ( normalize_U2 ) normalizeU2();
      trace.endBlock();
      trace.beginBlock("Solving for v");
      former_v0 = v0;
      PrimalForm1 squared_norm_d_u2 = PrimalForm1::zeros(calculus);
      for ( unsigned int d = 0; d < 3; ++d )
          squared_norm_d_u2.myContainer.array() += (primal_AD2*u2[d]).myContainer.array().square();
      const PrimalIdentity0 ope_v0 = l_1_over_4e_Id0 + lambda*epsilon*primal_D0.transpose()*primal_D0 + M01.transpose()*dec_helper::diagonal(squared_norm_d_u2)*M01;

      trace.info() << "Prefactoring matrix V" << std::endl;
      SolverV0 solver_v0;
      solver_v0.compute( ope_v0 );
      trace.info() << "Solving V v = l/4e * 1" << std::endl;
      v0 = solver_v0.solve( l_1_over_4e );
      trace.info() << "  => " << ( solver_v0.isValid() ? "OK" : "ERROR" )
                   << " " << solver_v0.myLinearAlgebraSolver.info() << std::endl;
      trace.endBlock();
    }

    void
    normalizeU2()
    {
      for ( Index index = 0; index < nb(2); index++)
        {
          double n2 = 0.0;
          for ( unsigned int d = 0; d < 3; ++d )
            n2 += u2[ d ].myContainer( index ) * u2[ d ].myContainer( index );
          double norm = sqrt( n2 );
          if (norm == 0) continue;
          for ( unsigned int d = 0; d < 3; ++d )
            u2[ d ].myContainer( index ) /= norm;
        }
    }

    void
    checkV0()
    {
      const double m1 = v0.myContainer.minCoeff();
      const double m2 = v0.myContainer.maxCoeff();
      const double ma = v0.myContainer.mean();
      trace.info() << "0-form v: min=" << m1 << " avg=" << ma << " max=" << m2 << std::endl;
    }

    void
    diffV0( double& n_infty, double& n_2, double& n_1 )
    {
      PrimalForm0 delta = v0-former_v0;
      delta.myContainer = delta.myContainer.cwiseAbs();

      n_infty = delta.myContainer.maxCoeff();
      n_2 = std::sqrt(delta.myContainer.squaredNorm()/delta.myContainer.size());
      n_1 = delta.myContainer.mean();
    }

    PrimalForm2
    getV2() const
    {
      return M12*M01*v0;
    }

    PrimalForm1
    getV1() const
    {
      return M01*v0;
    }

    void
    updateSmallestEpsilonMap(SmallestEpsilonMap& smallest_eps, const double threshold = .5) const
    {
        const KSpace& K = calculus.myKSpace;
        for ( const SCell face_signed : calculus.getIndexedSCells<2, PRIMAL>() )
        {
            const Cell face            = K.unsigns( face_signed );
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
            features[ 0 ] = v0.myContainer( calculus.getCellIndex( p00 ) );
            features[ 1 ] = v0.myContainer( calculus.getCellIndex( p01 ) );
            features[ 2 ] = v0.myContainer( calculus.getCellIndex( p10 ) );
            features[ 3 ] = v0.myContainer( calculus.getCellIndex( p11 ) );
            std::sort( features.begin(), features.end() );

            if ( features[ 1 ] <= threshold )
            {
                auto it = smallest_eps.find( face );
                if ( it != smallest_eps.end() ) it->second = std::min( epsilon, it->second );
                else smallest_eps[ face ] = epsilon;
            }
        }
    }

    
    
    /**
     * Default constructor.
     */
    SurfaceATSolver() = delete;

    /**
     * Destructor.
     */
    ~SurfaceATSolver() = delete;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    SurfaceATSolver ( const SurfaceATSolver & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    SurfaceATSolver ( SurfaceATSolver && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    SurfaceATSolver & operator= ( const SurfaceATSolver & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    SurfaceATSolver & operator= ( SurfaceATSolver && other ) = delete;

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

    /// Initializes the operators
    void initOperators()
    {
      if ( verbose >= 1 ) trace.beginBlock( "[SurfaceATSolver::initOperators] Solver initialization" );
      if ( verbose >= 2 ) trace.info() << "derivative of primal 0-forms: primal_D0" << std::endl;
      primal_D0 = calculus.derivative<0,PRIMAL>();
      if ( verbose >= 2 ) trace.info() << "derivative of primal 1-forms: primal_D1" << std::endl;
      primal_D1 = calculus.derivative<1,PRIMAL>();
      if ( verbose >= 2 ) trace.info() << "antiderivative of primal 2-forms: primal_AD2" << std::endl;
      primal_AD2 = calculus.antiderivative<2,PRIMAL>();
      if ( verbose >= 2 ) trace.info() << "vertex to edge average operator: M01" << std::endl;
      M01       = primal_D0;
      M01.myContainer = .5 * M01.myContainer.cwiseAbs();
      if ( verbose >= 2 ) trace.info() << "edge to face average operator: M12" << std::endl;
      M12       = primal_D1;
      M12.myContainer = .25 * M12.myContainer.cwiseAbs();
      if ( verbose >= 1 ) trace.endBlock();
    }
    
    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class SurfaceATSolver


  /**
   * Overloads 'operator<<' for displaying objects of class 'SurfaceATSolver'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'SurfaceATSolver' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const SurfaceATSolver<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/SurfaceATSolver.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SurfaceATSolver_h

#undef SurfaceATSolver_RECURSES
#endif // else defined(SurfaceATSolver_RECURSES)
