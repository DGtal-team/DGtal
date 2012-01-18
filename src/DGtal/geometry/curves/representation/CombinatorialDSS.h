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
 * @file CombinatorialDSS.h
 * @brief Dynamical recognition of DSS on FreemanChain code. 
 * @author Xavier Provençal (\c xavier.provencal@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/04/29
 *
 * Header file for module CombinatorialDSS.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see testCombinDSS.cpp
 */

#if defined(CombinatorialDSS_RECURSES)
#error Recursive header files inclusion detected in CombinatorialDSS.h
#else // defined(CombinatorialDSS_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CombinatorialDSS_RECURSES

#if !defined CombinatorialDSS_h
/** Prevents repeated inclusion of headers. */
#define CombinatorialDSS_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/curves/representation/FreemanChain.h"
#include "DGtal/geometry/curves/representation/ArithmeticalDSS.h"
#include "DGtal/base/OrderedAlphabet.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class CombinatorialDSS
  /**
   * Description of template class 'CombinatorialDSS' <p>
   * \brief Aim:
   *
   * A combinatorial DSS is a specialized type of DSS where input points
   * are given by a Freeman Chain code.
   *
   * This class uses the fact that the part of a Freeman chain code that,
   * in general, is a DSS has the following form : 's.c^k.p' where 'k>0',
   * 'c' is a Christoffel word, 's' is a suffix of 'c' and 'p' a prefix
   * of 'c'.
   *
   * More precisely 'c' codes the path between two consecutive upper
   * leaning points so the only exceptions are where the DSS is parallel
   * to one of the axes , in this case the DSS is called 'trivial', and
   * when the DSS has only one upper leaning point.
   *
   * This class is a model of the concept CSegmentComputer.
   *
   * The main interest of this class is to provide the function
   * 'longestChristoffelPrefix'. A Christoffel word is a particular type
   * of pattern in a 4-connected DSS such that both the starting point
   * and the end point are upper leaning points (see Berstel, Lauve,
   * Reutenauer and Saliola [2008]).
   *
   * @tparam TInteger the type of scalars used for the coordinates of the
   * points (satisfying CInteger) 
   */


  template <typename TInteger>
    class CombinatorialDSS  
  {

    // ----------------------- Types ------------------------------
    public :
      //Required type
      BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );
      typedef TInteger Integer;

      //Required types
      typedef FreemanChain<TInteger> FreemanChainCode;
      typedef typename FreemanChainCode::ConstIterator ConstIterator;
      typedef CombinatorialDSS<Integer> Self;

      //2D points and 2D vectors
        // \TODO : récupérer Point et Vector a partir de FreemanChain
      typedef DGtal::PointVector<2,Integer> Point;
      typedef DGtal::PointVector<2,Integer> Vector;


      typedef char Code;
      typedef int Size;
      typedef int Index;

      //Arithmetical DSS, for comparaison purpose
      typedef DGtal::ArithmeticalDSS< ConstIterator, Integer, 4 > ArithmeticalDSS;

    // ----------------------- Standard services ------------------------------
    public:

      /**
       * Default constructor
       */
      CombinatorialDSS(){}

      /**
       * Destructor.
       */
      ~CombinatorialDSS();

      /**
       * Initialize from a ConstIterator over a Freman Chain code.
       * @param ConstIterator giving the letter to initialize the DSS with.
       */
      CombinatorialDSS(const ConstIterator& it);

      /**
       * Initialize from a ConstIterator over a Freman Chain code.
       * @param Iterator giving the letter to initialize the DSS with.
       */
      void init(const ConstIterator& it);

      /**
       * Copy constructor.
       * @param other the object to clone.
       */
      CombinatorialDSS ( const Self & other );

      /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       */
      CombinatorialDSS & operator= ( const Self & other );

      /**
       * Equality operator.
       * @param other the object to compare with.
       */
      bool operator==( const Self & other ) const;

      /**
       * Difference operator.
       * @param other the object to compare with.
       * @return 'false' if equal
       * 'true' otherwise
       */
      bool operator!=( const Self & other ) const;

      /**
       * Equatlity operator, comparing with ArithmericalDSS
       * @param other the object to compare with.
       */
      bool operator==( const ArithmeticalDSS & other) const;

      /**
       * Difference operator, comparing with ArithmeticalDSS.
       * @param other the object to compare with.
       * @return 'false' if equal
       * 'true' otherwise
       */
      bool operator!=( const ArithmeticalDSS & other ) const;

      /**
       * Tests whether the current DSS can be extended at the front or at
       * the back depending on the iterator given as argument.  Computes
       * the parameters of the extended DSS if yes.
       * @return 'true' if yes, 'false' otherwise.
       */
      bool extendForward(ConstIterator it);

      /**
       * Tests whether the current DSS can be extended at the front.
       * Computes the parameters of the extended DSS if yes.
       * @return 'true' if yes, 'false' otherwise.
       */
      bool extendForward();

      /**
       * Tests whether the current DSS can be extended at the back.
       * Computes the parameters of the extended DSS if yes.
       * @return 'true' if yes, 'false' otherwise.
       */
      bool extendBackward();

      /**
       * Removes the first point of the DSS (at back). 
       * NB : Unlike the ArithmeticalDSS, a CombinatorialDSS must
       * containt at least two points since it is defined by a letter in
       * a Freeman Chain code.  
       * @return 'true' if the first point is removed, 'false' otherwise.
       */
      bool retractForward();

      /**
       * Removes the last point of the DSS (at front).
       * NB : Unlike the ArithmeticalDSS, a CombinatorialDSS must
       * containt at least two points since it is defined by a letter in
       * a Freeman Chain code.  
       * @return 'true' if the last point is removed, 'false' otherwise.
       */
      bool retractBackward();

      /**
       * Initializes the DSS with the longest Christoffel word
       * that is read starting from a given position on a
       * FreemanCode. Only Christoffel words written on a
       * given pair of letters are considered. More precisely,
       * if the ordered alphabet is [a0, a1, a2, a3] then the
       * Christoffel word must be written on the letter a1 < a2.
       *
       * Note that by usually Christoffel words are defined as
       * primitive words while here repetitions of a
       * Christoffel word will be included in the DSS.
       *
       * Computation time is O(k) where k is the number of
       * points included in the DSS.
       *
       * @param aFC a FreemanChain.
       *
       * @param aOA the ordered alphabet.
       *
       * @param len (returns) the number of points inserted in
       * the DSS which is exacly the length of the Christoffel
       * word read (with repetitions).
       *
       * @param s the position from where the FreemanCode is
       * read (default value = 0).
       *
       * @return 'true' if the FreemanChain is coding a path
       * that is possibly digitally convex, 'false' if the
       * path is not digitally convex.
       */ 
      bool longestChristoffelPrefix(
          ConstIterator it,
          const OrderedAlphabet & aOA);

      /**
       * Computes the arithmetic description of the DSS.
       * @param (returns) 'a' from the equation mu <= ax-by < mu + omega
       * @param (returns) 'b' from the equation mu <= ax-by < mu + omega
       * @param (returns) 'mu' from the equation mu <= ax-by < mu + omega
       * @param (returns) 'omega' from the equation mu <= ax-by < mu + omega
       */
      void getArithmeticalDescription( Integer &a, Integer &b, Integer
          &mu, Integer &omega) const;

      /**
       * Computes the arithmetic description of the DSS : 0 <= ax+by+mu < omega
       * @return the value of 'a' in the DSS equation
       */
      Integer getA() const;

      /**
       * Computes the arithmetic description of the DSS : 0 <= ax+by+mu < omega
       * @return the value of 'b' in the DSS equation
       */
      Integer getB() const;

      /**
       * Computes the arithmetic description of the DSS : 0 <= ax+by+mu < omega
       * @return the value of 'mu' in the DSS equation
       */
      Integer getMu() const;

      /**
       * Computes the arithmetic description of the DSS : 0 <= ax+by+mu < omega
       * @return the value of 'omega' in the DSS equation
       */
      Integer getOmega() const;

      /**
       * Accessor to the first upper leaning point
       * @return first upper leaning point.
       */
      Point getUf() const;

      /**
       * Accessor to the last upper leaning point
       * @return last upper leaning point.
       */
      Point getUl() const;

      /**
       * Accessor to the first lower leaning point
       * @return first lower leaning point.
       */
      Point getLf() const;

      /**
       * Accessor to the last lower leaning point
       * @return last lower leaning point.
       */
      Point getLl() const;

      /**
       * Performs some basic tests to check the validity of the DSS. 
       * For debugging purpose only.
       * @returns 'false' if the data is incoherent.
       */
      bool isValid() const;
      
      // ----------------------- Accessors --------------------------------------
      
    public:

      /**
       * Accessor to the first added point to the DSS
       * @return point.
       */
      Point getFirstPoint() const;

      /**
       * Accessor to the last added point to the DSS
       * @return point.
       */
      Point getLastPoint() const;

      /**
       * @return begin iterator of the DSS range.
       */
      ConstIterator begin() const;

      /**
       * @return end iterator of the DSS range.
       */
      ConstIterator end() const;






      // ----------------------- Interface --------------------------------------
    public:

      /**
       * Writes/Displays the object on an output stream.
       * @param out the output stream where the object is written.
       */
      void selfDisplay ( std::ostream & out ) const;


    protected:
      // ------------------------- Protected Datas ------------------------------
      
      /**
       * Freeman chain on which is defined the CombinatarialDSS
       */
      const FreemanChainCode * myFC;

      /**
       * In order to keep track of the DSS position, first and last points
       * are memorized.
       */
      Point myFirstPoint;
      Point myLastPoint;

      /**
       * Index of the first letter of DSS in the FreemanChain
       */
      Index myFirstLetter;
      Index myLastLetter;


      /**
       * Number of repetitions of the central Christoffel
       * word, otherwise said, the number of upper leaning
       * points minus 1.
       */
      unsigned int myNbRepeat;

      /**
       * Indexes indicates where, in the FreemanChain, is located the main
       * pattern of this DSS.
       */
      Index myPatternBegin;
      Index myPatternEnd;

      /**
       * The main pattern factorizes as two subpatterns: a
       * left one and a right one.
       */
      Size myLeftPatternLength;

      /**
       * In order to add/remove letters efficiently from the
       * prefix and the suffix of the main pattern being read, 
       * the position of the next letter to read is memorized.
       */
      Index myNextBefore;
      Index myNextAfter;



    private:
      // ------------------------- Private Datas --------------------------------
    private:

      // ------------------------- Hidden services ------------------------------
    protected:

      /**
       * Returns the first letter of the main pattern.
       * @returns the small letter over which the DSS is written.
       */
      Code getSmallLetter() const;

      /**
       * Returns the last letter of the main pattern.
       * @returns the big letter over which the DSS is written.
       */
      Code getBigLetter() const;

      /** 
       * Get the code at a given position in the
       * FreemanChain
       * @param a position in the FreemanChain
       * @returns the letter at the given position
       */
      Code getCode(Index pos) const;

      /**
       * Computes the length of the main pattern.
       * @returns the length of the main pattern
       */
      Size mainPatternLength() const;

      /**
       * Computes the length of the suffix of the main pattern read before
       * it. 
       * @returns the length of the suffix read.
       */
      Size suffixLength() const;

      /**
       * Computes the length of the prefix of the main pattern read after
       * it. 
       * @returns the length of the prefix read.
       */
      Size prefixLength() const;

      /**
       * Determines if a given position is a "upper leaning
       * point". Note that it is assume that the orientation
       * is such that the main pattern begins and ends on
       * upper leaning points.
       * NB: 'pos' must be between 'myPatternBegin' and 'myPatternEnd'
       * @param the position of a letter in the main pattern of the DSS
       * @returns 'true' if this letter is an "upper leaning point"
       *   'false' otherwise.
       */
      bool isUL ( Index pos ) const;

      /**
       * Determines if the letter at a given position leads a "lower
       * leaning point". Note that it is assume that the orientation is
       * such that the main pattern begins and ends on upper leaning
       * points.
       * NB: 'pos' must be between 'myPatternBegin' and 'myPatternEnd'
       * @param the position of a letter in the main pattern of the DSS
       * @returns 'true' if this letter leads to a "lower leaning point"
       * 'false' otherwise.
       */
      bool nextIsLL ( Index pos ) const;

      /**
       * Determines if the letter at a given position comes from a
       * "lower leaning point". Note that it is assume that the
       * orientation is such that the main pattern begins and ends on
       * upper leaning points.
       * NB: 'pos' must be between 'myPatternBegin' and 'myPatternEnd'
       * @param the position of a letter in the main pattern of the DSS
       * @returns 'true' if this letter comes from a "lower leaning
       * point" 'false' otherwise.
       */
      bool previousIsLL ( Index pos ) const;

      /**
       * Test if the DSS is a trivial one, that is a DSS with slope 0 or
       * infinite
       * @returns 'true' is the DSS is trivial, 'false' otherwise.
       */
      bool isTrivial() const;


    
    private:
      // ------------------------- Internals ------------------------------------
    private:

  }; // end of class CombinatorialDSS


  /**
   * Overloads 'operator<<' for displaying objects of class 'CombinatorialDSS'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'CombinatorialDSS' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const CombinatorialDSS<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/representation/CombinatorialDSS.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CombinatorialDSS_h

#undef CombinatorialDSS_RECURSES
#endif // else defined(CombinatorialDSS_RECURSES)
