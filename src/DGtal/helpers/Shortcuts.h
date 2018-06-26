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
 * @date 2018/06/25
 *
 * Header file for module Shortcuts.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Shortcuts_RECURSES)
#error Recursive header files inclusion detected in Shortcuts.h
#else // defined(Shortcuts_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Shortcuts_RECURSES

#if !defined Shortcuts_h
/** Prevents repeated inclusion of headers. */
#define Shortcuts_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <cstdlib>
#include <iostream>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/io/readers/MPolynomialReader.h"
#include "DGtal/shapes/implicit/ImplicitPolynomial3Shape.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/helpers/Parameters.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  
  /////////////////////////////////////////////////////////////////////////////
  // template class Shortcuts
  /**
   * Description of template class 'Shortcuts' <p> \brief Aim: This
   * class is used to simplify shape and surface creation. With it,
   * you can create new shapes and surface in a few lines. The
   * drawback is that you use specific types or objects, which could
   * lead to faster code or more compact data structures.
   */
  template  < typename TKSpace >
  class Shortcuts
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));

    // ----------------------- Usual space types --------------------------------------
  public:
    typedef TKSpace                                  KSpace;
    typedef typename KSpace::Space                   Space;
    typedef typename Space::Integer                  Integer;
    typedef typename Space::Point                    Point;
    typedef typename Space::Vector                   Vector;
    typedef typename Space::RealVector               RealVector;
    typedef typename Space::RealPoint                RealPoint;
    typedef typename RealVector::Component           Scalar;
    typedef HyperRectDomain<Space>                   Domain;

    // ----------------------- Shortcut types --------------------------------------
  public:
    /// defines a multi-variate polynomial : RealPoint -> Scalar
    typedef MPolynomial< Space::dimension, Scalar >  ScalarPolynomial;
    /// defines an implicit shape of the space, which is the
    /// zero-level set of a ScalarPolynomial.
    typedef ImplicitPolynomial3Shape<Space>          ImplicitShape;
    /// defines the digitization of an implicit shape.
    typedef GaussDigitizer< Space, ImplicitShape >   ImplicitDigitalShape;
     

    // ----------------------- Static services --------------------------------------
  public:

    /// @return the parameters and their default values used in shortcuts.
    /// - "polynomial" : "sphere1"
    static Parameters defaultParameters()
    {
      return parametersImplicitShape()
	| parametersKSpace()
	| parametersDigitization();
    }

    /// Returns a map associating a name and a polynomial,
    /// e.g. "sphere1", "x^2+y^2+z^2-1".
    ///
    /// { "sphere1", "x^2+y^2+z^2-1" },
    /// { "sphere9", "x^2+y^2+z^2-81" },
    /// { "ellipsoid", "3*x^2+2*y^2+z^2-90" },
    /// { "cylinder", "x^2+2*z^2-90" },
    ///	{ "torus",   "(x^2+y^2+z^2+6*6-2*2)^2-4*6*6*(x^2+y^2)" },
    /// { "rcube",   "x^4+y^4+z^4-6561" },
    /// { "goursat", "-1*(8-0.03*x^4-0.03*y^4-0.03*z^4+2*x^2+2*y^2+2*z^2)" },
    /// { "distel",  "10000-(x^2+y^2+z^2+1000*(x^2+y^2)*(x^2+z^2)*(y^2+z^2))"},
    /// { "leopold", "(x^2*y^2*z^2+4*x^2+4*y^2+3*z^2)-100" },
    /// { "diabolo", "x^2-(y^2+z^2)^2" },
    /// { "heart",   "-1*(x^2+2.25*y^2+z^2-1)^3+x^2*z^3+0.1125*y^2*z^3" },
    /// { "crixxi",  "-0.9*(y^2+z^2-1)^2-(x^2+y^2-1)^3" }
    ///
    /// @return the map associating a polynomial to a name.
    static std::map< std::string, std::string >
    polynomialList()
    {
      std::vector< std::pair< std::string, std::string > >
	Ps = { { "sphere1", "x^2+y^2+z^2-1" },
	       { "sphere9", "x^2+y^2+z^2-81" },
	       { "ellipsoid", "3*x^2+2*y^2+z^2-90" },
	       { "cylinder", "x^2+2*z^2-90" },
	       { "torus",   "(x^2+y^2+z^2+6*6-2*2)^2-4*6*6*(x^2+y^2)" },
	       { "rcube",   "x^4+y^4+z^4-6561" },
	       { "goursat", "-1*(8-0.03*x^4-0.03*y^4-0.03*z^4+2*x^2+2*y^2+2*z^2)" },
	       { "distel",  "10000-(x^2+y^2+z^2+1000*(x^2+y^2)*(x^2+z^2)*(y^2+z^2))"},
	       { "leopold", "(x^2*y^2*z^2+4*x^2+4*y^2+3*z^2)-100" },
	       { "diabolo", "x^2-(y^2+z^2)^2" },
	       { "heart",   "-1*(x^2+2.25*y^2+z^2-1)^3+x^2*z^3+0.1125*y^2*z^3" },
	       { "crixxi",  "-0.9*(y^2+z^2-1)^2-(x^2+y^2-1)^3" } };
      std::map< std::string, std::string > L;
      for ( auto p : Ps )
	L[ p.first ] = p.second;
      return L;
    }

    /// @return the parameters and their default values which are used
    /// to define an implicit shape.
    ///   - polynomial["sphere1"]: the implicit polynomial whose zero-level set
    ///                            defines the shape of interest.
    static Parameters parametersImplicitShape()
    {
      return Parameters( "polynomial", "sphere1" );
    }

    /// Builds a 3D implicit shape from parameters
    ///
    /// @param[in] params the parameters:
    ///   - polynomial["sphere1"]: the implicit polynomial whose zero-level set
    ///                            defines the shape of interest.
    ///
    /// @return a smart pointer on the created implicit shape.
    static CountedPtr<ImplicitShape>
    makeImplicitShape( const Parameters& params = parametersImplicitShape() )
    {
      typedef MPolynomialReader< Space::dimension, Scalar> Polynomial3Reader;
      std::string poly_str = params[ "polynomial" ].as<std::string>();
      // Recognizes specific strings as polynomials.
      auto PL = polynomialList();
      if ( PL[ poly_str ] != "" ) poly_str = PL[ poly_str ];
      ScalarPolynomial poly;
      Polynomial3Reader reader;
      std::string::const_iterator iter
	= reader.read( poly, poly_str.begin(), poly_str.end() );
      if ( iter != poly_str.end() )
	{
	  trace.error() << "[Shortcuts::makeImplicitShape]"
			<< " ERROR reading polynomial: I read only <"
			<< poly_str.substr( 0, iter - poly_str.begin() )
			<< ">, and I built P=" << poly << std::endl;
	}
      return CountedPtr<ImplicitShape>( new ImplicitShape( poly ) );
    }

    /// @return the parameters and their default values which are used for digitization.
    ///   - closed   [1]: specifies if the Khalimsky space is closed (!=0) or not (==0).
    static Parameters parametersKSpace()
    {
      return Parameters
	( "closed",  1 );
    }

    /// @return the parameters and their default values which are used for digitization.
    ///   - minAABB  [-10.0]: the min value of the AABB bounding box (domain)
    ///   - maxAABB  [ 10.0]: the max value of the AABB bounding box (domain)
    ///   - gridstep [  1.0]: the gridstep that defines the digitization (often called h).
    ///   - offset   [  5.0]: the digital dilation of the digital space,
    ///                       useful when you process shapes and that you add noise.
    static Parameters parametersDigitization()
    {
      return Parameters
	( "minAABB",  -10.0 )
	( "maxAABB",   10.0 )
	( "gridstep",   1.0 )
	( "offset",     5.0 );
    }

    /// Builds a Khalimsky space that encompasses the lower and upper
    /// digital points.  Note that digital points are cells of the
    /// Khalimsky space with maximal dimensions.  A closed Khalimsky
    /// space adds lower dimensional cells all around its boundary to
    /// define a closed complex.
    ///
    /// @param[in] params the parameters:
    ///   - closed   [1]: specifies if the Khalimsky space is closed (!=0) or not (==0).
    ///
    /// @return the Khalimsky space.
    static KSpace makeKSpace( const Point& low, const Point& up,
			      Parameters params = parametersKSpace() )
    {
      int closed  = params[ "closed"  ].as<int>();
      KSpace K;
      if ( ! K.init( low, up, closed ) )
	trace.error() << "[Shortcuts::makeKSpace]"
		      << " Error building Khalimsky space K=" << K << std::endl;
      return K;
    }
    
    /// Builds a Khalimsky space that encompasses the bounding box
    /// specified by \a params. It is for instance useful when
    /// digitizing an implicit shape.
    ///
    /// @param[in] params the parameters:
    ///   - minAABB  [-10.0]: the min value of the AABB bounding box (domain)
    ///   - maxAABB  [ 10.0]: the max value of the AABB bounding box (domain)
    ///   - gridstep [  1.0]: the gridstep that defines the digitization (often called h).
    ///   - offset   [  5.0]: the digital dilation of the digital space,
    ///                       useful when you process shapes and that you add noise.
    ///   - closed   [1]    : specifies if the Khalimsky space is closed (!=0) or not (==0).
    ///
    /// @return the Khalimsky space.
    /// @see makeImplicitDigitalShape
    static KSpace makeKSpaceDigitization( Parameters params =
					  parametersKSpace() | parametersDigitization() )
    {
      Scalar min_x  = params[ "minAABB"  ].as<Scalar>();
      Scalar max_x  = params[ "maxAABB"  ].as<Scalar>();
      Scalar h      = params[ "gridstep" ].as<Scalar>();
      Scalar offset = params[ "offset"   ].as<Scalar>();
      bool   closed = params[ "closed"   ].as<int>();
      RealPoint p1( min_x - offset * h, min_x - offset * h, min_x - offset * h );
      RealPoint p2( max_x + offset * h, max_x + offset * h, max_x + offset * h );
      CountedPtr<ImplicitDigitalShape> dshape( new ImplicitDigitalShape() );
      dshape->init( p1, p2, h );
      Domain domain = dshape->getDomain();
      KSpace K;
      if ( ! K.init( domain.lowerBound(), domain.upperBound(), closed ) )
	trace.error() << "[Shortcuts::makeKSpace]"
		      << " Error building Khalimsky space K=" << K << std::endl;
      return K;
    }

    /// Makes the Gauss digitization of the given implicit shape
    /// according to parameters. Use makeKSpace to build the associated digital space.
    ///
    /// @param[in] shape a smart pointer on the implicit shape.
    /// @param[in] params the parameters:
    ///   - minAABB  [-10.0]: the min value of the AABB bounding box (domain)
    ///   - maxAABB  [ 10.0]: the max value of the AABB bounding box (domain)
    ///   - gridstep [  1.0]: the gridstep that defines the digitization (often called h).
    ///   - offset   [  5.0]: the digital dilation of the digital space,
    ///                       useful when you process shapes and that you add noise.
    ///
    /// @return a smart pointer on the created implicit digital shape.
    /// @see makeKSpaceDigitization 
    static CountedPtr<ImplicitDigitalShape>
    makeImplicitDigitalShape
    ( CountedPtr<ImplicitShape> shape,
      Parameters params = parametersDigitization() )
    {
      Scalar min_x  = params[ "minAABB"  ].as<Scalar>();
      Scalar max_x  = params[ "maxAABB"  ].as<Scalar>();
      Scalar h      = params[ "gridstep" ].as<Scalar>();
      Scalar offset = params[ "offset"   ].as<Scalar>();
      RealPoint p1( min_x - offset * h, min_x - offset * h, min_x - offset * h );
      RealPoint p2( max_x + offset * h, max_x + offset * h, max_x + offset * h );
      CountedPtr<ImplicitDigitalShape> dshape( new ImplicitDigitalShape() );
      dshape->attach( shape );
      dshape->init( p1, p2, h );
      return dshape;
    }
    
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Default constructor.
     */
    Shortcuts() = delete;

    /**
     * Destructor.
     */
    ~Shortcuts() = delete;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    Shortcuts ( const Shortcuts & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    Shortcuts ( Shortcuts && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Shortcuts & operator= ( const Shortcuts & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    Shortcuts & operator= ( Shortcuts && other ) = delete;

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

  }; // end of class Shortcuts


  /**
   * Overloads 'operator<<' for displaying objects of class 'Shortcuts'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Shortcuts' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const Shortcuts<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/helpers/Shortcuts.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Shortcuts_h

#undef Shortcuts_RECURSES
#endif // else defined(Shortcuts_RECURSES)
