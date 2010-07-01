/**
 * @file Mathutils.cpp
 *
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 * Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 *
 * @date 2010/07/01
 *
 * Implementation of methods defined in Mathutils.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cstdlib>
#include <cmath>
#include "DGtal/math/Mathutils.h"
// Includes inline functions/methods if necessary.
#if !defined(INLINE)
#include "DGtal/math/Mathutils.ih"
#endif
///////////////////////////////////////////////////////////////////////////////

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// class Mathutils
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ------------------------- Static services ------------------------------

/**
 * 2*pi.
 */
const float 
DGtal::Mathutils::two_pi_f = (float) 2*M_PI;


/**
 * 2*pi.
 */
const double
DGtal::Mathutils::two_pi_d = (double) 2.0*M_PI;


/**
 * @param angle an angle in radian (arbitrary).
 * @return the equivalent angle in [0;2PI[.
 */
float
DGtal::Mathutils::angleMod( float angle )
{
  while ( angle < 0.0f ) angle += two_pi_f;
  while ( angle > two_pi_f ) angle -= two_pi_f;
  return angle;
}


/**
 * @param angle an angle in radian (arbitrary).
 * @return the equivalent angle in [-PI;PI[.
 */
float 
DGtal::Mathutils::angleAroundZero( float angle )
{
  while ( angle < -M_PI ) angle += two_pi_f;
  while ( angle > M_PI ) angle -= two_pi_f;
  return angle;
}


/**
 * @param angle_ref an angle in radian (arbitrary).
 * @param other_angle an other angle in radian (arbitrary).
 * @return the angle equivalent to [other_angle] but expressed in a window [-PI;PI[ wrt [angle_ref].
 */
float
DGtal::Mathutils::tune( float angle_ref, float other_angle )
{
  return angleAroundZero( other_angle - angle_ref ) + angle_ref;
}



/**
 * @param angle1 an angle in radian (arbitrary).
 * @param angle2 an angle in radian (arbitrary).
 * @return 'true' if angle1 strictly precedes angle2 when both are brought back in the same window of size PI.
 */
bool 
DGtal::Mathutils::less( float angle1, float angle2 )
{
  float diff = angleAroundZero( angle2 - angle1 );
  return diff > 0.0f;
}


/**
 * Average length of an angle t: dl(t)=1/(|cos(t)|+|sin(t)|).
 * @param t an angle in radian (arbitrary).
 * @return the corresponding "averaged length" (discrete geometry).
 */
float
DGtal::Mathutils::averagedLength( float t )
{
  return 1.0f / ( fabs( cos( t ) ) + fabs( sin( t ) ) );
}


///////////////////////////////////////////////////////////////////////////////
// ------------------------- arithmetic services -----------------------------

/**
 * Returns the simple continued fraction of a/b.
 * 
 * @param z (returns) the partial coefficients.
 * @param a the numerator
 * @param b the denominator
 *
 * @return the gcd of a and b.
 */
uint
DGtal::Mathutils::cfrac( vector<uint> & z, uint a, uint b )
{
  while ( b != 0 )
    {
      uint q = a / b;
      uint r = a - b * q;
      z.push_back( q );
      a = b;
      b = r;
    }
  return a;
}

/**
 * Returns the greatest common divisor of a and b.
 * 
 * @param a the numerator (possibly negative).
 * @param b the denominator (possibly negative).
 *
 * @return the gcd of a and b.
 */

int 
DGtal::Mathutils::gcd( int a, int b )
{
  if ( a < 0 ) a = -a;
  if ( b < 0 ) b = -b;
  while ( b != 0 )
    {
      int q = a / b;
      int r = a - b * q;
      a = b;
      b = r;
    }
  return a;
}



///////////////////////////////////////////////////////////////////////////////
// ------------------------- random services -----------------------------


/**
 * @param p any integer below 2^31.
 * @return a random number between 0 and p-1.
 */
unsigned int
DGtal::Mathutils::random( unsigned int p )
{
  return (unsigned int) ( ( (double) p ) * rand() / ( RAND_MAX+1.0 ) );
}


/**
 * @param p any integer below 2^63.
 * @return a random number between 0 and p-1.
 */
unsigned long long 
DGtal::Mathutils::random( unsigned long long p )
{
  if ( p > 0xffffffffffffLL )
    {
      unsigned int p2 = ( (unsigned int) ( p >> 48 ) ) & 0xffffff;
      unsigned int q0 = DGtal::Mathutils::random( (unsigned int) 0x1000000 );
      unsigned int q1 = DGtal::Mathutils::random( (unsigned int) 0x1000000 );
      unsigned int q2 = DGtal::Mathutils::random( p2 );
      return (unsigned long long) q0
	+ ( ( (unsigned long long) q1 ) << 24 )
	+ ( ( (unsigned long long) q2 ) << 48 );
    }
  else if ( p > 0xffffff )
    {
      unsigned int p1 = ( (unsigned int) ( p >> 24 ) ) & 0xffffff;
      unsigned int q0 = DGtal::Mathutils::random( (unsigned int) 0x1000000 );
      unsigned int q1 = DGtal::Mathutils::random( p1 );
      return (unsigned long long) q0
	+ ( ( (unsigned long long) q1 ) << 24 );
    }
  else
    {
      unsigned int p0 = ( (unsigned int) p ) & 0xffffff;
      unsigned int q0 = DGtal::Mathutils::random( p0 );
      return (unsigned long long) q0;
    }
}


/**
 * @return a uniform random value between 0.0 (included) and (1.0) excluded.
 */
double 
DGtal::Mathutils::random1()
{
  return ((double) rand())/(RAND_MAX+1.0);
}



///////////////////////////////////////////////////////////////////////////////
// ------------------------- bit services ------------------------------


/**
 * NB: Relatively constant time.
 * @param v any value.
 * @return the most-significant bit set to 1 in [v], (0) if 0
 */
int
DGtal::Mathutils::getMSBbyLog( unsigned long long v )
{
  register int i;
  if ( v & 0xffffffff00000000LL )
    {
      i = 32;
      v >>= i;
    }
  else i = 0;

  i += (v & 0xffff0000) ? 16 : 0;

  if ( (v >>= i) & 0xff00 )
    i |= 8, v >>= 8;
  if ( v & 0xf0 )
    i |= 4, v >>= 4;
  if ( v & 0xc )
    i |= 2, v >>= 2;
  return ( i | ( v >> 1 ) );
}

/**
 * Calls 'getMSBbyLog'.
 * @param v any value.
 * @return the most-significant bit set to 1 in [v], (0) if 0
 */
int 
DGtal::Mathutils::getMSB( unsigned long long v )
{
  return getMSBbyLog( v );
}



///////////////////////////////////////////////////////////////////////////////
// ------------------------- prime services ------------------------------


/**
 * @param n any positive integer.
 * @return a prime number greater than [n].
 */
unsigned long long
DGtal::Mathutils::greaterPrime( unsigned long long n )
{
  //BK
  //TODO
  //ASSERT_Mathutils( n != 0 );
  uint i = getMSB( n - 1 ) + 1;
  

  //BK
  //TODO
  //ASSERT_Mathutils( i < m_size_primes );
  return m_primes[ i ];
}




///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/**
 * Destructor. 
 */
DGtal::Mathutils::~Mathutils()
{
}

///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
 * Writes/Displays the object on an output stream.
 * @param that_stream the output stream where the object is written.
 */
void 
DGtal::Mathutils::selfDisplay( ostream& that_stream ) const
{
  that_stream << "[Mathutils]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool 
DGtal::Mathutils::OK() const
{
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Internals - private :


/**
 * An array of prime number such that m_primes[ i ] >= 2^i
 */
const unsigned long long 
DGtal::Mathutils::m_primes[ 33 ] =
  { 2, 2, 5, 11, 17, 37, 67, 131, // 2^0 --> 2^7
    257, 521, 1031, 2053, 4099, 8209, 16411, 32771, // 2^8 --> 2^15
    65537, 131101, 262147, 524309, // 2^16 --> 2^19
    1048583, 2097169, 4194319, 8388617, // 2^20 --> 2^23
    16777259, 33554467, 67108879, 134217757, // 2^24 --> 2^27
    268435459, 536870923, 1073741827, 2147483659LL, // 2^28 --> 2^31
    4294967311LL // 2^32
  };


