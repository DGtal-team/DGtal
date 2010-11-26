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
//LICENSE-END
#pragma once

/**
 * @file GreedyDecomposition.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/10/25
 *
 * Header file for module GreedyDecomposition.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GreedyDecomposition_RECURSES)
#error Recursive header files inclusion detected in GreedyDecomposition.h
#else // defined(GreedyDecomposition_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GreedyDecomposition_RECURSES

#if !defined GreedyDecomposition_h
/** Prevents repeated inclusion of headers. */
#define GreedyDecomposition_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
	
  /////////////////////////////////////////////////////////////////////////////
  // template class GreedyDecomposition
  /**
   * Description of template class 'GreedyDecomposition' <p>
   * \brief Aim: Computes the greedy decomposition of a contour into primitives
   * (the last point of the primitive i is the first point of the primitive i+1).

   * The contour must provide a constIterator as a way of scanning its points 
   * and must be able to return its size.
   * The primitive must have a method addFront() to check whether the primitive
   * can be extended at the front or not.  
   
   * Here is an example of how to use this class to decompose a contour into DSSs :
   * @code 
   
   
   typedef int Coordinate;
   typedef PointVector<2,Coordinate> Point;
   
   //Define the primitive as a standard DSS
   typedef ArithmeticalDSS<StandardBase<Coordinate> > PrimitiveType;
   
   // Define the contour as a FreemanChain
   typedef FreemanChain<Coordinate> ContourType; 
   
   // Open the contour file
   std::string filename = "myContour.fc";
   std::fstream fst;
   fst.open (filename.c_str(), std::ios::in);
   ContourType theContour(fst);
   
   // Decomposition of the contour into DSSs
   GreedyDecomposition<ContourType,PrimitiveType> theDecomposition(theContour);

   * @endcode
   */
  template <typename Contour, typename Primitive>
  class GreedyDecomposition
  {

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~GreedyDecomposition();

    /**
     * This class is an iterator on the contour, 
     * storing the current primitive.
     */
    class ConstIterator
    {
      	   
			   // ------------------------- data -----------------------
    private:
      /**
       * The contour visited by the iterator.
       */
      const Contour* myC;


      /**
       * An iterator of the contour 
       * at the front of the current primitive
       */
      typename Contour::ConstIterator myFront;

      /**
       * An iterator of the contour 
       * at the back of the current primitive
       */
      typename Contour::ConstIterator myBack;

      /**
       * The current position in the contour.
       */
      unsigned int myPosition;

      /**
       * The current primitive of the iterator.
       */
      Primitive  myP;
      


      // ------------------------- Standard services -----------------------
    public:
       friend class GreedyDecomposition<Contour,Primitive>;
			   
		 /**
       * Default Constructor.
       * The object is not valid.
       */
      ConstIterator();

      /**
       * Constructor.
       * Nb: complexity in O(n).
       *
       * @param aContour a contour
			 * @param n, the position in the sequence of primitives
       * (Set at aContour.size() for the end iterator).
       */
      ConstIterator( const Contour & aContour, unsigned int n );

      /**
       * Copy constructor.
       * @param other the iterator to clone.
       */
      ConstIterator( const ConstIterator & aOther );
    
      /**
       * Assignment.
       * @param aOther the iterator to copy.
       * @return a reference on 'this'.
       */
      ConstIterator& operator=( const ConstIterator & aOther );
    
      /**
       * Destructor. Does nothing.
       */
      ~ConstIterator();
    
      // ------------------------- iteration services -------------------------
    public:
      
      /**
       * @return the current primitive
       */
      Primitive operator*() const;

      /**
       * @return the current primitive.
       */
      Primitive get() const;

      /**
       * Pre-increment.
       * Goes to the next primitive on the contour (if possible).
       * Nb: complexity in O(n).
       */
      ConstIterator& operator++();
      
      /**
       * Goes to the next primitive on the contour (if possible).
       * Nb: complexity in O(n).
       */
      void next();


      /**
       * Pre-decrement.
       * Goes to the previous primitive on the chain (if possible).
       * Nb: complexity in O(n).
       */
      ConstIterator& operator--();
      
      /**
       * Goes to the previous primitive on the chain (if possible).
       * Nb: complexity in O(n).
       */
      void previous();

      /**
       * @return the current position 
       * (as an index in the sequence of primitives).
       */
      unsigned int getPosition() const;

      /**
       * @return the associated Contour.
       */
      const Contour* getContour() const;

      /**
       * @return an iterator of the contour
       * at the front of the primitive.
       */
      const typename Contour::ConstIterator getFront() const;

      /**
       * @return an iterator of the contour
       * at the back of the primitive.
       */
      const typename Contour::ConstIterator getBack() const;

      /**
       * Equality operator.
       *
       * @param aOther the iterator to compare with 
       * (must be defined on the same contour).
       *
       * @return 'true' if their current positions coincide.
       */
      bool operator==( const ConstIterator & aOther ) const;

      /**
       * Inequality operator.
       *
       * @param aOther the iterator to compare with 
       * (must be defined on the same contour).
       *
       * @return 'true' if their current positions differs.
       */
      bool operator!=( const ConstIterator & aOther ) const;

      /**
       * Inferior operator.
       *
       * @param aOther the iterator to compare with 
       * (must be defined on the same contour).
       *
       * @return 'true' if the current position of 'this' is before
       * the current position of [other].
       */
      bool operator<( const ConstIterator & aOther ) const;
      
    };


    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Constructor.
     * @param aContour, the contour whose decomposition is computed
     */
    GreedyDecomposition(const Contour& aContour);

    /**
     * Iterator service.
     * @return an iterator pointing on the first primitive of the contour.
     */
    typename GreedyDecomposition::ConstIterator begin() const;

    /**
     * Iterator service.
     * @return an iterator pointing after the last primitive of the contour.
     */
    typename GreedyDecomposition::ConstIterator end() const;


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
  private:
    // ------------------------- Private Datas --------------------------------
  private:

		const Contour* myContour;

    // ------------------------- Hidden services ------------------------------


  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    GreedyDecomposition ( const GreedyDecomposition & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    GreedyDecomposition & operator= ( const GreedyDecomposition & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class GreedyDecomposition


  /**
   * Overloads 'operator<<' for displaying objects of class 'GreedyDecomposition'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'GreedyDecomposition' to write.
   * @return the output stream after the writing.
   */
  template <typename Contour, typename Primitive>
  std::ostream&
  operator<< ( std::ostream & out, const GreedyDecomposition<Contour, Primitive> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/GreedyDecomposition.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GreedyDecomposition_h

#undef GreedyDecomposition_RECURSES
#endif // else defined(GreedyDecomposition_RECURSES)
