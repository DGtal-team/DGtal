/**
 * @file approximation.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/02/06
 *
 * An example file named approximation.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
//! [approximation-basicIncludes]
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <iomanip>
#include "DGtal/arithmetic/LighterSternBrocot.h"
#include "DGtal/base/StdRebinders.h"
//! [approximation-basicIncludes]

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

void usage( int, char** argv )
{
  std::cerr << "Usage: " << argv[ 0 ] << " <floating point number>" << std::endl;
  std::cerr << "\t - computes the successive rational approximation of this number, up to a precision of 1e-14." << std::endl;
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

  //! [approximation-types]
  typedef DGtal::int64_t Integer;
  typedef DGtal::int64_t Quotient;
  typedef LighterSternBrocot<Integer, Quotient, StdMapRebinder> SB; // the type of the Stern-Brocot tree
  typedef SB::Fraction Fraction; // the type for fractions
  typedef std::back_insert_iterator< Fraction > OutputIterator;
  //! [approximation-types]

  //! [approximation-process]
  long double epsilon = 1e-14;
  long double number0 = strtold( argv[ 1 ], 0 );
  long double number = number0;
  ASSERT( number >= 0.0 );
  Fraction f;
  OutputIterator itback = std::back_inserter( f );
  Quotient i = 0;
  while ( true )
    {
      long double int_part = floorl( number );
      Quotient u = NumberTraits<long double>::castToInt64_t( int_part );
      *itback++ = std::make_pair( u, i++ );
      long double approx =
        ( (long double) NumberTraits<Integer>::castToDouble( f.p() ) )
        / ( (long double) NumberTraits<Integer>::castToDouble( f.q() ) );
      std::cout << "z = " << f.p() << " / " << f.q()
                << " =~ " << std::setprecision( 16 ) << approx << std::endl;
      number -= int_part;
      if ( ( (number0 - epsilon ) < approx )
           && ( approx < (number0 + epsilon ) ) ) break;
      number = 1.0 / number;
    }
  std::cout << "z = " << f.p() << " / " << f.q() << std::endl;
  //! [approximation-process]
  return 0;
}

