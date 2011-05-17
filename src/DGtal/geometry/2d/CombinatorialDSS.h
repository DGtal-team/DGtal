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
 * @author Xavier Provençal (\c xavier.provencal@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/04/29
 *
 * Header file for module CombinatorialDSS.cpp
 *
 * This file is part of the DGtal library.
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
#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "DGtal/geometry/2d/FreemanChain.h"
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
	 * A combinatorial DSS is a particular type of arithmetical DSS
	 * specialized in the case where the input points are given by a Freeman
	 * Chain code.  
	 *
	 * This class inherits from the class ArithmeticalDSS with the following
	 * template arguments are fixed : 'TIterator' is
	 * 'DGtal::FreemanChain<TInteger>::ConstIterator' and 'connectivity' is
	 * 4.
	 *
	 * The main interest of this class is to provide the function
	 * 'longestChristoffelPrefix'. A Christoffel word a particular type of
	 * pattern in a 4-connected DSS such that both the starting point and
	 * the end point are upper leaning points (see Berstel, Lauve,
	 * Reutenauer and Saliola [2008]).
	 *
	 * This class is templated by the typename 'TInteger', the type of the
	 * coordinates of the points (satisfying CInteger).
	 * 
	 */


	template <typename TInteger>
		class CombinatorialDSS : 
			public ArithmeticalDSS<
			typename FreemanChain<TInteger>::ConstIterator,
			TInteger, 
			4>
	{

		// ----------------------- Types ------------------------------
		public :
			typedef typename FreemanChain<TInteger>::ConstIterator Iterator;
			typedef FreemanChain<TInteger> FreemanChainCode;
			typedef TInteger Integer;
			typedef ArithmeticalDSS<Iterator, Integer, 4> DSS;
			typedef typename DSS::Point Point;
			typedef typename DSS::Vector Vector;

		// ----------------------- Standard services ------------------------------
		public:

			/**
			 * Default constructor
			 * Does nothing!
			 */
			CombinatorialDSS(){}

			/**
			 * Destructor.
			 */
			~CombinatorialDSS(){}



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
					Iterator it,
					const OrderedAlphabet & aOA);



			// ----------------------- Accessors --------------------------------------
		public:

			/**
			 * Iterator service.
			 * @return an iterator pointing on the first point of the chain.
			 */
			Iterator begin()
			{
				return this->myF;
			}


			/**
			 * Iterator service.
			 * @return an iterator pointing on the last point of the chain.
			 */
			Iterator end()
			{
				return this->myL;
			}



			// ----------------------- Interface --------------------------------------
		public:

			/**
			 * Writes/Displays the object on an output stream.
			 * @param out the output stream where the object is written.
			 */
			void selfDisplay ( std::ostream & out ) const;


			// ------------------------- Protected Datas ------------------------------
		private:
			// ------------------------- Private Datas --------------------------------
		private:

			// ------------------------- Hidden services ------------------------------
		protected:

			
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
#include "DGtal/geometry/2d/CombinatorialDSS.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CombinatorialDSS_h

#undef CombinatorialDSS_RECURSES
#endif // else defined(CombinatorialDSS_RECURSES)
