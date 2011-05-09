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
 * @file testCombinDSS.cpp
 * @ingroup Tests
 * @author Xavier Provençal (\c xavier.provencal@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/05/02
 *
 * Functions for testing class CombinDSS.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/2d/CombinatorialDSS.h"
#include "DGtal/geometry/2d/ArithmeticalDSS.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CombinDSS.
///////////////////////////////////////////////////////////////////////////////


/**
 * Builds a CombinatorialDSS and the corresponding ArithmeticalDSS and compares
 * bots
 *
 * Note : afterward, the iterator is positionned at the end of the DSS.
 *
 * @param it an iterator on a FreemanChain<int>
 * @param aAlph the corresponding OrderedAlphabet
 * @return 'true' if both DSS are equals, 'false' otherwise.
 */
bool compareDSS(FreemanChain<int>::ConstIterator & it, const OrderedAlphabet & aAlph)
{
	CombinatorialDSS<int> CDSS;
	CDSS.longestChristoffelPrefix(it, aAlph);
	ArithmeticalDSS<FreemanChain<int>::ConstIterator, int, 4> ADSS(it);
	do {
		ADSS.extend(++it);
	} while (it != CDSS.end());
	return CDSS == ADSS;
}



/**
 * Builds CombinatorialDSS and ArithmeticalDSS in the fourth quadrants and
 * compares them, if both are equals, the test is passed.
 *
 */
bool testCombinDSS()
{

	unsigned int nbok = 0;
	unsigned int nb = 0;

	trace.beginBlock ( "Testing DuvalPPtoDSS first quadrant" );
	{
		OrderedAlphabet A('0', 4);
		A.shiftRight();
		FreemanChain<int> chain( "001010010100011011101111", 10, 10);
		FreemanChain<int>::ConstIterator it = chain.begin();
		while (it.getPosition()+1 != chain.end().getPosition() ) {
			nb++;
			nbok += compareDSS(it, A) ? 1 : 0;
			trace.info() << "(" << nbok << "/" << nb << ") "
				<< "true == true" << std::endl;
		}
	}
	trace.endBlock();

	trace.beginBlock ( "Testing DuvalPPtoDSS second quadrant" );
	{
		OrderedAlphabet A('0', 4);
		FreemanChain<int> chain( "1221112212121211221122", 10, 10);
		FreemanChain<int>::ConstIterator it = chain.begin();
		while (it.getPosition()+1 != chain.end().getPosition() ) {
			nb++;
			nbok += compareDSS(it, A) ? 1 : 0;
			trace.info() << "(" << nbok << "/" << nb << ") "
				<< "true == true" << std::endl;
		}
	}
	trace.endBlock();

	trace.beginBlock ( "Testing DuvalPPtoDSS third quadrant" );
	{
		OrderedAlphabet A('0', 4);
		A.shiftLeft();
		FreemanChain<int> chain( "2323222332232322323", 10, 10);
		FreemanChain<int>::ConstIterator it = chain.begin();
		while (it.getPosition()+1 != chain.end().getPosition() ) {
			nb++;
			nbok += compareDSS(it, A) ? 1 : 0;
			trace.info() << "(" << nbok << "/" << nb << ") "
				<< "true == true" << std::endl;
		}
	}
	trace.endBlock();

	trace.beginBlock ( "Testing DuvalPPtoDSS fourth quadrant" );
	{
		OrderedAlphabet A('0', 4);
		A.shiftRight();
		A.shiftRight();
		FreemanChain<int> chain( "00003303003003003300", 20, 21);
		FreemanChain<int>::ConstIterator it = chain.begin();
		while (it.getPosition()+1 != chain.end().getPosition() ) {
			nb++;
			nbok += compareDSS(it, A) ? 1 : 0;
			trace.info() << "(" << nbok << "/" << nb << ") "
				<< "true == true" << std::endl;
		}
	}
	trace.endBlock();
	return nbok == nb;
}

bool testEquality()
{
	unsigned int nbok = 0;
	unsigned int nb = 0;

	CombinatorialDSS<int> CDSS1, CDSS2, CDSS3;
	trace.beginBlock ( "Testing equality of Combinatorial DSS" );
	{
		OrderedAlphabet A('0', 4);
		FreemanChain<int> chain1(   "11121112112", -1, 1);
		FreemanChain<int> chain2( "1211121112112", 0, 0);

		FreemanChain<int>::ConstIterator it1 = chain1.begin();
		CDSS1.longestChristoffelPrefix( it1, A );

		FreemanChain<int>::ConstIterator it2 = chain2.begin();
		CDSS2.longestChristoffelPrefix( it2, A );
		it2 = CDSS2.end();
		CDSS3.longestChristoffelPrefix( it2, A );
	}
	nb++;
	nbok += ( (CDSS1 != CDSS2 ) && ( CDSS1 == CDSS3 ) ) ? 1 : 0;
	trace.info() << "(" << nbok << "/" << nb << ") "
		<< "true == true" << std::endl;
	trace.endBlock();
	return nbok == nb;
}




///////////////////////////////////////////////////////////////////////////////
// Standard services - public :
int main( int argc, char** argv )
{
	trace.beginBlock ( "Testing class CombinDSS" );
	trace.info() << "Args:";
	for ( int i = 0; i < argc; ++i )
		trace.info() << " " << argv[ i ];
	trace.info() << endl;

	bool res = testCombinDSS() && testEquality(); // && ... other tests
	trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
	trace.endBlock();

	return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
