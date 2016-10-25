/**
 * @file math/polynomial-derivative.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/02/06
 *
 * An example file named polynomial-derivative.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
//! [polynomial-derivative-basicIncludes]
#include <iostream>
#include <string>
#include <sstream>
#include "DGtal/math/MPolynomial.h"
#include "DGtal/io/readers/MPolynomialReader.h"
//! [polynomial-derivative-basicIncludes]

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

void usage( int, char** argv )
{
  std::cerr << "Usage: " << argv[ 0 ] << " <P>" << std::endl;
  std::cerr << "\t - computes the first and second derivative of the given polynomial P (in one variable)." << std::endl;
}

/**
   Main.
*/
int main( int argc, char** argv )
{
  if ( argc < 2 )
    {
      usage( argc, argv );
      return 1;
    }

  //! [polynomial-derivative-types]
  typedef double Ring;
  typedef MPolynomial<1, Ring> MyPolynomial;
  //! [polynomial-derivative-types]

  //! [polynomial-derivative-process]
  std::string polynomialString( argv[ 1 ] );
  std::istringstream polynomialIStream( polynomialString );
  MyPolynomial P;
  polynomialIStream >> P;
  MyPolynomial P1 = derivative<0>( P );
  MyPolynomial P2 = derivative<0>( P1 );
  std::cout << "P(X_0)   = " << P << std::endl;
  std::cout << "P'(X_0)  = " << P1 << std::endl;
  std::cout << "P''(X_0) = " << P2 << std::endl;
  //! [polynomial-derivative-process]
  return 0;
}

