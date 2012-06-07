/**
 * @file convergents-biginteger.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/02/06
 *
 * An example file named convergents-biginteger.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
//! [convergents-biginteger-basicIncludes]
#include <iostream>
#include "DGtal/arithmetic/LighterSternBrocot.h"
//! [convergents-biginteger-basicIncludes]

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

void usage( int, char** argv )
{
  std::cerr << "Usage: " << argv[ 0 ] << " <p> <q>" << std::endl;
  std::cerr << "\t - computes the successive convergent of the fraction p / q." << std::endl;
}

/**
   Main.
*/
int main( int argc, char** argv )
{
  if ( argc < 3 )
    {
      usage( argc, argv );
      return 1;
    }
  std::string inputP = argv[ 1 ];
  std::string inputQ = argv[ 2 ];

  //! [convergents-biginteger-types]
  typedef BigInteger Integer;
  typedef DGtal::int64_t Quotient;
  typedef LighterSternBrocot<Integer, Quotient, StdMapRebinder> SB; // the type of the Stern-Brocot tree
  typedef SB::Fraction Fraction; // the type for fractions
  typedef Fraction::ConstIterator ConstIterator; // the iterator type for visiting quotients
  typedef Fraction::Value Value; // the value of the iterator, a pair (quotient,depth).
  //! [convergents-biginteger-types]

  //! [convergents-biginteger-instantiation]
  Integer p( inputP );
  Integer q( inputQ );
  Fraction f( p, q ); // fraction p/q
  //! [convergents-biginteger-instantiation]

  //! [convergents-biginteger-cfrac]
  // Visit quotients u_k as pair (u_k,k)
  std::cout << "z = ";
  ConstIterator itbegin = f.begin(), itend = f.end();
  for ( ConstIterator it = itbegin; it != itend; ++it )
    {
      Value u = *it;
      std::cout << ( ( it == itbegin ) ? "[" : "," )
                << u.first;
    }
  std::cout << "]" << std::endl;
  //! [convergents-biginteger-cfrac]

  //! [convergents-biginteger-convergents]
  Fraction g;         // fraction null, 0/0, invalid
  for ( ConstIterator it = itbegin; it != itend; ++it )
    {
      Value u = *it;
      std::cout << "z_" << u.second << " = ";
      g.push_back( u ); // add (u_i,i) to existing fractions
      std::cout << g.p() << " / " << g.q() << std::endl;
    }
  //! [convergents-biginteger-convergents]
  return 0;
}

