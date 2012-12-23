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
 * This class is a model of the concept CDynamicBidirectionalSegmentComputer. 
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
#include "DGtal/geometry/curves/FreemanChain.h"
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
   * A combinatorial DSS is a specialized type of 4-connected DSS that reads
   * codes of a Freeman chain as input.
   * 
   * In general, the Freeman coding of a 4-connected DSS has the following form : 
   * 's.c^k.p' where 'k>0', 'c' is a Christoffel word, 's' is a suffix of 'c'
   * and 'p' a prefix of 'c'.
   *
   * More precisely 'c' codes the path between two consecutive upper
   * leaning points so the only exceptions are where the DSS is parallel
   * to one of the axes , in this case the DSS is called 'trivial', and
   * when the DSS has only one upper leaning point.
   *
   * This class is a model of the concept CBidirectionalSegmentComputer.
   *
   * @tparam TConstIterator the type of iterator used to read the input codes
   * (preferably of category 'random_access_iterator_tag').
   * @tparam TInteger the type of scalars used for the coordinates of the
   * points (satisfying CInteger) 
   */


  template <typename TConstIterator, typename TInteger>
    class CombinatorialDSS  
  {

    // ----------------------- Types ------------------------------
    public :
      //Required type
      BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );
      typedef TConstIterator ConstIterator;
      typedef TInteger Integer;

      //Required types
      typedef FreemanChain<TInteger> FreemanChainCode;
      typedef CombinatorialDSS<ConstIterator, Integer> Self;
      typedef CombinatorialDSS<ConstIterator, Integer> Reverse;

      //2D points and 2D vectors
      typedef DGtal::PointVector<2,Integer> Point;
      typedef DGtal::PointVector<2,Integer> Vector;


      typedef typename iterator_traits<ConstIterator>::value_type Code;
      typedef int Size;
      typedef int Index;

      //The basic steps associate to the codes are given by a function `f: Code -> Vector` 
      typedef Vector (*DisplacementFct) (Code);


    private :
      /**
       * CodeHandler encapsulates an iterator in order to provide constant time
       * access to the codes previously read.
       * @tparam TIterator an iterator on the codes.
       * @tparam iterator_type the type of iterations services provided by TIterator.
       */ 
      template < class TIterator, class iterator_type = typename iterator_traits<TIterator>::iterator_category >
      class CodeHandler
        {
        public :
          CodeHandler()
            { }
          void init( const TIterator & it ) 
            { 
              myIter = it;
            }

          Code getCode( Index n ) const
            {
              return myCodes[ n ];
            }

          Code getCode( Index n )
            {
              while ( n >= myCodes.size() )
                {
                  myCodes.push_back( *myIter );
                  ++myIter;
                }
              return myCodes[ n ];
            }

        private :
          vector<Code> myCodes;
          TIterator myIter;
        };

      /**
       * Partial specialization template in the case where the iterator is of
       * cartegory bidirectional
       */
      template < class TIterator >
      class CodeHandler< TIterator, bidirectional_iterator_tag > 
        {
        public :
          CodeHandler()
            {
            }
          void init( const TIterator & it ) 
            { 
              myFirst = it;
              myLast = it;
            }

          Code getCode( Index n ) const
            {
              return ( n >= 0 ) ? myPosCodes[ n ] : myNegCodes[ 1-n ];
            }

          Code getCode( Index n )
            {
              Code c;
              if ( n<0 )
                {
                  unsigned int i = 1-n;
                  while ( i >= myNegCodes.size() )
                    {
                      --myFirst;
                      myNegCodes.push_back( *myFirst );
                    }
                  c = myNegCodes[ n ];
                }
              else
                {
                  unsigned int i = n;
                  while ( i >= myPosCodes.size() )
                    {
                      myPosCodes.push_back( *myLast );
                      ++myLast;
                    }
                  c =  myPosCodes[ n ];
                }
              return c;
            }

        private :
          vector<Code> myPosCodes;
          vector<Code> myNegCodes;
          TIterator myFirst;
          TIterator myLast;
        };

      /**
       * Partial template specialization for random access iterators.
       */ 
      template < class TIterator>
        class CodeHandler<TIterator, random_access_iterator_tag >
          {
          public :
            CodeHandler()
              { }

            void init ( const TIterator & it ) 
            { 
              myIter = it;
            }

            Code getCode( Index n ) const
              {
                return myIter[ n ];
              }

            Code getCode ( Index n )
              {
                return myIter[ n ];
              }

          private :
            TIterator myIter;
          };


    public :

      /**
       * Iterator on the points of the DSS
       */
      struct ConstPointIterator
        {

          typedef bidirectional_iterator_tag iterator_category;
          typedef Point value_type;
          typedef Index difference_type;
          typedef Point * pointer;
          typedef Point & reference;

          const CombinatorialDSS * myDSS;
          Index i;
          Point p;

          /**
           * Default constructor, does nothing
           */
          ConstPointIterator( )
          {}

          /**
           * Initialization constructor.
           * @param Index of the first letter.
           * @param CombinatorialDSS on which the iterator is defined.
           * @param Starting point of the iterator.
           */
          ConstPointIterator( const CombinatorialDSS * dss, Index ind, Point pt ) :
            myDSS(dss), i(ind), p(pt) 
          {}

          /**
           * Destructor. Does nothing.
           */
          ~ConstPointIterator() {}
            
          /**
           * Comparaison operators.
           */
          bool operator==( const ConstPointIterator other) const
            {
              return i == other.i;
            }
          bool operator!=( const ConstPointIterator other) const
            {
              return i != other.i;
            }

          Index operator-( const ConstPointIterator other) const
            {
              return i - other.i;
            }


          /**
           * Copy operator
           */
          ConstPointIterator& operator=( const ConstPointIterator & other)
            {
              i = other.i;
              myDSS = other.myDSS;
              p = other.p;
              return *this;
            }

          Point operator*() const
            {
              return p;
            }

          // pre-increment ++i
          ConstPointIterator& operator++()
            {
              next();
              return *this;
            }

          // post-increment
          ConstPointIterator operator++( int )
            {
              ConstPointIterator it = *this;
              next();
              return it;
            }


          // pre-decrement --i
          ConstPointIterator& operator--()
            {
              prev();
              return *this;
            }

          // post-decrement
          ConstPointIterator operator--( int )
            {
              ConstPointIterator it = *this;
              prev();
              return it;
            }

          // Move to next position
          void next()
            {
              p += myDSS->myDisplacements( myDSS->getCode( i ) );
              ++i;
            }

          // Move to previous position
          void prev()
            {
              --i;
              p -= myDSS->myDisplacements( myDSS->getCode( i ) );
            }


          // Get a reference to the current Combinatorial DSS
          const CombinatorialDSS * getDSS() const
            {
              return myDSS;
            }


          // Get the index of the iterator
          Index getIndex() const
            {
              return i;
            }

        };




    // ----------------------- Standard services ------------------------------
    public:

      /**
       * Default constructor
       */
      CombinatorialDSS();

      /**
       * Destructor.
       */
      ~CombinatorialDSS();


      /**
       * Initialize from input iterator. A DSS of length 1 is initialize from
       * the iterator.
       *
       * By default, displacements are defined as : 
       * '0' -> (1,0), '1' -> (0,1), '2' -> (-1,0), '3' -> (0,-1)
       *
       * @param itFirst the first code to include in the DSS.
       * @oaram start the position where the DSS starts.
       * @param displacement, the function that defines displacement vectors
       * from codes.
       */
      void init( const ConstIterator & it, 
                const Point & start = Point(0,0),
                Vector (*displacements) (Code) = defaultMoves );



      /**
       * Initialize from a ConstPointIterator on a CombinatorialDSS.
       * @param ConstPointIterator
       */
      void init(const ConstPointIterator & i);

      /**
       * Initialize from a Freman Chain code.
       *
       * Note : to be used, this initialization method requires that the class
       * is templated by string::const_iterator.
       *
       * @param FreemanChain on which is defined the DSS.
       */
      void init(const FreemanChainCode & fc);

      /**
       * Initialize from a ConstIterator over a Freman Chain code.
       *
       * Note : to be used, this initialization method requires that the class
       * is templated by string::const_iterator.
       *
       * @param ConstIterator giving the letter to initialize the DSS with.
       */
      void init(const typename FreemanChainCode::ConstIterator & it);

      ///**
      // * Initialize from an array of codes.
      // * @param a pointer to an array of codes.
      // * @param the length of the code array.
      // * @param the start position in 'theCode'.
      // * @param the position of the starting point of the first code.
      // * @param a function that returns the displacement vector of the codes.
      // */
      //void init(const Code * theCode, Size codeLength, Index firstLetter, const
      //          Point & startPoint, Vector (*displacement) (Code) );

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
       * @returns an uninitialized instance of CombinatorialDSS.
       */
      Self getSelf( ) const;



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
       * @return a default-constructed instance of Reverse
       */
      Reverse getReverse() const;


      /**
       * Tests whether the current DSS can be extended at the front.
       *  
       * @return 'true' if yes, 'false' otherwise.
       */
      bool isExtendableForward();

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
       * Tests whether the current DSS can be extended at the back.
       *  
       * @return 'true' if yes, 'false' otherwise.
       */
      bool isExtendableBackward();

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
       * Set the position of the first point of the DSS.
       * @param p the point where the DSS starts;
       */
      void setPosition( const Point & p );


      /**
       * Translates the DSS's position by a given vector.
       * @param v the translation vector.
       */
      void translate( const Vector & v );

      /**
       * Computes the arithmetic description of the DSS : 0 <= ax+by+mu < omega
       * @param (returns) 'a' from the equation mu <= ax-by < mu + omega
       * @param (returns) 'b' from the equation mu <= ax-by < mu + omega
       * @param (returns) 'mu' from the equation mu <= ax-by < mu + omega
       * @param (returns) 'omega' from the equation mu <= ax-by < mu + omega
       */
      void getArithmeticalDescription( Integer &a, Integer &b, Integer
          &mu, Integer &omega) const;

      /**
       * Computes the arithmetic description of the DSS : 0 <= ax+by+mu < omega
       * Uses 'getArithmeticalDescription' so prefer this latter one if more
       * then one parameter is computed.
       *
       * @return the value of 'a' in the DSS equation
       */
      Integer getA() const;

      /**
       * Computes the arithmetic description of the DSS : 0 <= ax+by+mu < omega
       * Uses 'getArithmeticalDescription' so prefer this latter one if more
       * then one parameter is computed.
       * 
       * @return the value of 'b' in the DSS equation
       */
      Integer getB() const;

      /**
       * Computes the arithmetic description of the DSS : 0 <= ax+by+mu < omega
       * Uses 'getArithmeticalDescription' so prefer this latter one if more
       * then one parameter is computed.
       *
       * @return the value of 'mu' in the DSS equation
       */
      Integer getMu() const;

      /**
       * Computes the arithmetic description of the DSS : 0 <= ax+by+mu < omega
       * Uses 'getArithmeticalDescription' so prefer this latter one if more
       * then one parameter is computed.
       *
       * @return the value of 'omega' in the DSS equation
       */
      Integer getOmega() const;


      /**
       * Computes the remained of a point relatively to the arithmetical
       * description of the current DSS.
       * @param aPoint a point whose remainder is returned
       * @returns the remaindre of aPoint
       */
      Integer getRemainder(const Point & aPoint) const;

      /**
       * Computes the leaning points of the DSS
       * @param (returns) the first upper leaning point.
       * @param (returns) the last upper leaning point.
       * @param (returns) the first lower leaning point.
       * @param (returns) the last lower leaning point.
       */
      void computeLeaningPoints( Point & uf, Point & ul, 
                                 Point & lf, Point & ll ) const;


      /**
       * Accessor to the first upper leaning point
       * Uses 'computeLeaningPoints' so prefer this latter one if more then one
       * leaning point is computed.
       * @return first upper leaning point.
       */
      Point getUf() const;

      /**
       * Accessor to the last upper leaning point
       * Uses 'computeLeaningPoints' so prefer this latter one if more then one
       * leaning point is computed.
       * @return last upper leaning point.
       */
      Point getUl() const;

      /**
       * Accessor to the first lower leaning point
       * Uses 'computeLeaningPoints' so prefer this latter one if more then one
       * leaning point is computed.
       * @return first lower leaning point.
       */
      Point getLf() const;

      /**
       * Accessor to the last lower leaning point
       * Uses 'computeLeaningPoints' so prefer this latter one if more then one
       * leaning point is computed.
       * @return last lower leaning point.
       */
      Point getLl() const;

      /**
       * Performs some basic tests to check the validity of the DSS. 
       * For debugging purpose only.
       * @returns 'false' if the data is incoherent.
       */
      bool isValid() const;

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
       * Accessor to the first added point to the DSS
       * @return point.
       */
      ConstPointIterator pointBegin() const;

      /**
       * Accessor to the last added point to the DSS
       * @return point.
       */
      ConstPointIterator pointEnd() const;

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
       * @no longer used
       * Freeman chain on which is defined the CombinatarialDSS
       */
      //const FreemanChainCode * myFC;

      /**
       * The array of char on which is defined the CombinatorialDSS
       */
      CodeHandler<ConstIterator> myCodeHandler;

      /**
       * Iterators on the input codes.
       */
      ConstIterator myBegin;
      ConstIterator myEnd;

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


      /**
       * Function that converts the codes of a FreemanChain into a elementary translations.
       */
      Vector (*myDisplacements) ( Code ) ;



      // ------------------------- Private Datas --------------------------------
    private:

      // ------------------------- Hidden services ------------------------------
    public:


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
       * Get the code at a given index, if code at index 'pos' has not been read
       * yet, the input iterator will be used to access it.
       * @param a position in the FreemanChain
       * @returns the letter at the given position
       */
      Code getCode(Index pos);

      /** 
       * Get the code at a given index.
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
       * Computes the vector defined by the main pattern.
       * @returns the vector defined by the main pattern.
       */
      Vector mainPatternVector() const;

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

      /**
       * Convert a code into vector.
       * @param a code.
       * @returns the vector defined by that code.
       */
      Vector displacement( Code c ) const;



    
    private:
      // ------------------------- Internals ------------------------------------
    public :

      /**
       * Default displacement vectors associated to codes.
       * 
       *        (1)
       *         ^
       *         |
       *         |
       * (2) <-------> (0)
       *         |
       *         |
       *         v
       *        (3)
       *
       */
      static Vector defaultMoves( Code c )
        {
          Integer x = ( c == '0' ) ? 1 : ( ( c == '2' ) ? -1 : 0 ) ;
          Integer y = ( c == '1' ) ? 1 : ( ( c == '3' ) ? -1 : 0 ) ;
          return Vector( x, y );
        }



  }; // end of class CombinatorialDSS


  /**
   * Overloads 'operator<<' for displaying objects of class 'CombinatorialDSS'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'CombinatorialDSS' to write.
   * @return the output stream after the writing.
   */
  template <typename T1, typename T2>
  std::ostream&
  operator<< ( std::ostream & out, const CombinatorialDSS<T1,T2> & object );



} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/CombinatorialDSS.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CombinatorialDSS_h

#undef CombinatorialDSS_RECURSES
#endif // else defined(CombinatorialDSS_RECURSES)
