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

/**
 * @file OrderedAlphabet.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 * @author Laurent Provot (\c Laurent.Provot@loria.fr )
 * LORIA (CNRS, UMR 7503), Nancy University, France
 *
 * @date 2010/07/01
 *
 * Implementation of methods defined in OrderedAlphabet.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/base/OrderedAlphabet.h"
#include "DGtal/arithmetic/ModuloComputer.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// class OrderedAlphabet
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/**
 * Destructor.
 */
DGtal::OrderedAlphabet::~OrderedAlphabet()
{
  if ( myOrder != 0 )
    delete[ ] myOrder;
}

/**
 * @return the current ordered alphabet.
 */
string
DGtal::OrderedAlphabet::orderedAlphabet() const
{
  char *tbl;
  tbl = (char *)malloc((myNb + 1)*sizeof(char));
  for ( unsigned int i = 0; i < myNb; ++i )
    {
      tbl[ myOrder[ i ] ] = myFirst + i;
    }
  tbl[ myNb ] = '\0';
  string s( tbl );
  free(tbl);
  return s;
}

/**
 * Shift a0 < a1 < ... < an to a1 < ... < an < a0
 */
void
DGtal::OrderedAlphabet::shiftLeft()
{
  unsigned int* p = myOrder;
  unsigned int* q = myOrder + myNb;
  while ( p != q )
    {
      unsigned int k = *p;
      *p = ( k == 0 ) ? myNb - 1 : k - 1; 
      ++p;
    }
}

/**
 * Shift a0 < a1 < ... < an to an < a0 < ... < an-1
 */
void 
DGtal::OrderedAlphabet::shiftRight()
{
  unsigned int* p = myOrder;
  unsigned int* q = myOrder + myNb;
  while ( p != q )
    {
      unsigned int k = *p + 1;
      *p = ( k == myNb ) ? 0 : k; 
      ++p;
    }
}

/**
 * Reverse the order a0 < a1 < ... < an to an < ... < a1 < a0
 */
void 
DGtal::OrderedAlphabet::reverse()
{
  unsigned int* p = myOrder;
  unsigned int* q = myOrder + myNb;
  while ( p != q )
    {
      *p = myNb - 1 - *p;
      ++p;
    }
}

/**
 * Reverse the order a0 < a1 < ... < an to a3 < a2 < a1 < a0 < an < ... 
 */
void 
DGtal::OrderedAlphabet::reverseAround12()
{
  unsigned int* p = myOrder;
  unsigned int* q = myOrder + myNb;
  while ( p != q )
    {
      *p = ( myNb + 3 - *p ) % myNb;
      ++p;
    }
}




/**
 * Gives the first lyndon factor of the word [w] starting at
 * position [s] in this alphabet.
 *
 * @param len (returns) the length of the primitive Lyndon factor
 * (which starts at position s).
 *
 * @param nb (returns) the number of times the Lyndon factor appears.
 * 
 * @param w a word
 * @param s the starting index in [w].
 * @param e the index after the end in [w] (s<e).
 */
void 
DGtal::OrderedAlphabet::firstLyndonFactor
( size_t & len, size_t & nb,
  const std::string & w, 
  index_t s, index_t e ) const
{
  index_t i = s;
  index_t j = s+1;
  while ( ( j < e ) && ( lessOrEqual( w[ i ], w[ j ] ) ) )
    {
      if ( equal( w[ i ], w[ j ] ) )
        ++i;
      else 
        i = s;
      ++j;
    }
  len = (size_t) j - i;
  nb = ( (size_t) ( j - s ) ) / len;
}


/**
 * Gives the first lyndon factor of the cyclic word [w]
 * starting at position [s] in this alphabet.
 *
 * @param len (returns) the length of the primitive Lyndon factor
 * (which starts at position s).
 *
 * @param nb (returns) the number of times the Lyndon factor appears.
 * 
 * @param w a word
 * @param s the starting index in [w].
 * @param e the index after the end in [w] (s and e arbitrary).
 */
void
DGtal::OrderedAlphabet::firstLyndonFactorMod
( size_t & len, size_t & nb,
  const std::string & w, 
  index_t s, index_t e ) const
{
  size_t modulo = (DGtal::OrderedAlphabet::size_t)w.size();
  ModuloComputer< Integer > mc( modulo );
  index_t i = s;
  index_t j = mc.next( s );
  while ( ( j != e ) && ( lessOrEqual( w[ i ], w[ j ] ) ) )
    {
      if ( equal( w[ i ], w[ j ] ) )
        mc.increment( i );
      else 
        i = s;
      mc.increment( j );
    }
  len = j >= i ? (size_t) ( j - i )
    : (size_t) ( j + modulo - i );
  nb = ( (size_t) ( ( j + modulo - s ) % modulo ) ) / len;
}


/**
 * @param len (returns) the length of the primitive Lyndon factor
 * (which starts at position s).
 *
 * @param nb (returns) the number of times the Lyndon factor appears.
 * 
 * @param w a word which starts with a1 or a2 at position s.
 * @param s the starting index in [w].
 * @param e the index after the end in [w] (s<e).
 */
bool DGtal::OrderedAlphabet::duvalPP( size_t & len, size_t & nb, 
    const std::string & w, index_t s, index_t e) const
{
  ASSERT( ( order( w[ s ] ) == 1 ) || ( order( w[ s ] ) == 2 ) );
  index_t i = s;
  index_t j = s+1;
  index_t p = s;
  index_t q = s+1;
  while ( ( j < e ) && ( lessOrEqual( w[ i ], w[ j ] ) ) )
  {
    if ( equal( w[ i ], w[ j ] ) )
    {
      if ( j == q ) 
        q += (p-s+1);
      ++i;
    }
    else
    {
      if ( ( j != q ) || ( order ( w[ j ] ) != 2 ) )
      {
        len = j-s; nb = 0;
        return false;
      }
      index_t tmp = p; 
      p = q;
      q += q - tmp;
      i = s;
    }
    ++j;
  }
  len = (size_t) j - i;
  nb = ( (size_t) (j-s) ) / len;
  return true;
}

/**
 * @param len (returns) the length of the primitive Lyndon factor
 * (which starts at position s).
 *
 * @param nb (returns) the number of times the Lyndon factor appears.
 *
 * @param n1 (returns) the number of occurrences of the letter a1
 * in the Lyndon factor
 *
 * @param n2 (returns) the number of occurrences of the letter a2
 * in the Lyndon factor
 *
 * @param Lf1 (returns) the number of occurrences of the letter a1
 * from 's' to the first lower leaning point.
 *
 * @param Lf2 (returns) the number of occurrences of the letter a2
 * from 's' to the first lower leaning point.
 * 
 * @param w a word which starts with a1 or a2 at position s.
 * @param s the starting index in [w].
 * @param e the index after the end in [w] (s<e).
 */
bool
DGtal::OrderedAlphabet::duvalPPtoDSS
( size_t & len, size_t & nb,
  unsigned int & n1,  unsigned int & n2, 
  unsigned int & lf1, unsigned int & lf2,
  const std::string & w, 
  index_t s, index_t e
  ) const
{
  ASSERT( ( order( w[ s ] ) == 1 ) || ( order( w[ s ] ) == 2 ) );
  index_t i = s;
  index_t j = s+1;
  index_t p = s;
  index_t q = s+1;
  unsigned int slope1 = (order( w[ i ] ) == 1) ? 1 : 0;
  unsigned int slope2 = (order( w[ i ] ) == 2) ? 1 : 0;
  lf1 = n1 = slope1;
  lf2 = n2 = slope2;
  nb = 1;
  
  while ( ( j < e ) && ( lessOrEqual( w[ i ], w[ j ] ) ) ) {

    //This 'if/else if' is added to compute the vector defined by
    //the Christoffel word, this is usefull in order to compute the
    //leaning points.
    if (order( w[ j ] ) == 1)
      ++slope1;
    else if (order( w[ j ] ) == 2)
      ++slope2;

    if ( equal( w[ i ], w[ j ] ) ) {
      if ( j == q ) {
        q += (p-s+1);

      //A repetition is read when j==s+(nb+1)*(p-s+1)-1
      } 
      if ( j == nb * (p-s+1) + p ) {
        ++nb;
      }
      ++i;
    } else {
      if ( ( j != q ) || ( order ( w[ j ] ) != 2 ) ) {
        break;
      }
      index_t tmp = p; 
      p = q;
      q += q - tmp;
      i = s;

      // Extra information to compute the leaning points
      lf1 = lf1 + (nb-1)*n1;
      lf2 = lf2 + (nb-1)*n2;
      n1 = slope1;
      n2 = slope2;
      nb = 1;
    }
    ++j;
  }
  len = (size_t) j - i;

  if ( nb != (size_t) (j-s) / len)
    cout << "ASSERT(" << nb << " == (" << j << "-" << s << ") / "<<len << ")" << endl;
  ASSERT( nb == (size_t) (j-s) / len);
  return true;
}


/**
 * Adaptation of Duval's algorithm to extract the first Lyndon factor
 * (FLF). Whilst scanning the Lyndon factor, it also checks whether it
 * is a Christoffel word or not. It returns 'true' if the FLF is
 * indeed a Christoffel word, otherwise returns false. It starts the
 * extraction at position [s] in the cyclic word [w].
 *
 * The alphabet takes the form a0 < a1 < a2 < ... < an-1. [w] starts
 * with a1 or a2 at position s.
 *
 * See [Provencal, Lachaud 2009].
 *
 * @param len (returns) the length of the primitive Lyndon factor
 * (which starts at position s).
 *
 * @param nb (returns) the number of times the Lyndon factor appears.
 * 
 * @param w a (cyclic) word which starts with a1 or a2 at position s.
 *
 * @param s the starting index in [w].
 * @param e the index after the end in [w] (s and e arbitrary).
 */
bool 
DGtal::OrderedAlphabet::duvalPPMod( size_t & len, size_t & nb,
               const std::string & w, 
               index_t s, index_t e ) const
{
  ASSERT( ( order( w[ s ] ) == 1 )
          || ( order( w[ s ] ) == 2 ) );
  size_t modulo = (DGtal::OrderedAlphabet::size_t)w.size();
  ModuloComputer< Integer > mc( modulo );
  ModuloComputer< Integer >::UnsignedInteger i = s;
  index_t j = mc.next( s );
  unsigned int p = 1;
  unsigned int q = 2;
  while ( ( j != e ) && ( lessOrEqual( w[ i ], w[ j ] ) ) )
    {
      if ( equal( w[ i ], w[ j ] ) )
        {
          if ( j == mc.cast( s + q - 1 ) )
            q += p;
          mc.increment( i );
        }
      else 
        {
          if ( ( j != mc.cast( s + q - 1 ) ) || ( order ( w[ j ] ) != 2 ) )
            {
              len = j; nb = 0;
              return false;
            }
          unsigned int tmp = p; 
          p = q;
          q += q - tmp;
          i = s;
        }
      mc.increment( j );
    }
  len = j >= i ? (size_t) ( j - i )
    : (size_t) ( j + modulo - i );
  nb = ( (size_t) ( ( j + modulo - s ) % modulo ) ) / len;
  return true;
}


///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
 * Writes/Displays the object on an output stream.
 * @param out the output stream where the object is written.
 */
void
DGtal::OrderedAlphabet::selfDisplay ( std::ostream & out ) const
{
    out << "[OrderedAlphabet]";
    out << " " << orderedAlphabet() << endl;
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool
DGtal::OrderedAlphabet::isValid() const
{
    return true;
}



///////////////////////////////////////////////////////////////////////////////
// ----------------------- MLP services ------------------------------

/**
 * Extracts the next edge of the minimum length polygon starting from
 * position [s] on the word [w]. The alphabet may be modified
 * (reversed or shifted). The output alphabet is some a0 < a1 < a2 < ...
 *
 * @param nb_a1 (returns) the number of letters a1 in the extracted edge (a1
 * in the output alphabet)
 *
 * @param nb_a2 (returns) the number of letters a2 in the extracted edge (a2
 * in the output alphabet)
 *
 * @param w the input (cyclic) word (may be modified in the process).
 *
 * @param s the starting point in the word (updated).
 *
 * @param cvx (updates) this boolean is flipped only if a change of
 * convexity is detected.
 *
 * @return the number of letters of the extracted edge.
 */ 
DGtal::OrderedAlphabet::size_t
DGtal::OrderedAlphabet::nextEdge( size_t & nb_a1,
            size_t & nb_a2,
            std::string & w,
            index_t & s,
            bool & cvx )
{
  ModuloComputer< Integer > mc( (const unsigned int)w.size() );
  size_t l;
  size_t len;
  size_t nb;
  bool inC = duvalPPMod( len, nb, w, s, s );
  if ( ! inC ) 
    // case : change of convexity
    {
      // JOL : temporary change of letter w[ s ]
      char tmp = w[ s ];
      index_t tmp_s = s;
      w[ s ] = letter( 2 ); // a3
      this->reverseAround12();
      cvx = ! cvx;
      l = nextEdge( nb_a1, nb_a2, w, s, cvx );
      // JOL : former letter is put back in string.
      w[ tmp_s ] = tmp;
    }
  else if ( ( len == 1 ) && ( order( w[ s ] ) == 1 ) ) 
    // case u=a1 => Quadrant change
    {
      this->shiftRight();
      s = mc.cast( s + nb );
      nb_a1 = 0;
      nb_a2 = nb - 1;
      l = nb;
    }
  else 
    { // standard (convex) case.
      l = len * nb;
      char a2 = letter( 2 ); 
      nb_a1 = len;
      nb_a2 = 0;
      index_t ss = s;
      s = mc.cast( s + l );
      while ( len != 0 )
        {
          if ( w[ ss ] == a2 ) ++nb_a2;
          mc.increment( ss );
          --len;
        }
      nb_a1 -= nb_a2;
      nb_a1 *= nb;
      nb_a2 *= nb;
    }
  return l;
}


///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
