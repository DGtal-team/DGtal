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
 * @file testInteger.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 * @author David Coeurjolly (\c david.coeurjolly@cnrs.fr ), LIRIS (CNRS, UMR 5205),
 *
 * @date 2010/07/02
 *
 * Functions for testing class Integer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/CUnsignedNumber.h"

#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class Integer.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 *
 */
TEST_CASE( "Checking CInteger models")
{
  trace.beginBlock ( "Checking CInteger models ..." );
  BOOST_CONCEPT_ASSERT(( concepts::CInteger<int> ));
  BOOST_CONCEPT_ASSERT(( concepts::CInteger<DGtal::int16_t> ));

  trace.info() << "  - max int16 = " << NumberTraits<DGtal::int16_t>::max()
  << std::endl;
  trace.info() << "  - min int16 = " << NumberTraits<DGtal::int16_t>::min()
  << std::endl;
  trace.info() << "  - max int32 = " << NumberTraits<DGtal::int32_t>::max()
  << std::endl;
  trace.info() << "  - min int32 = " << NumberTraits<DGtal::int32_t>::min()
  << std::endl;
  trace.info() << "  - max int64 = " << NumberTraits<DGtal::int64_t>::max()
  << std::endl;
  trace.info() << "  - min int64 = " << NumberTraits<DGtal::int64_t>::min()
  << std::endl;
  trace.info() << "  - max uint16 = " << NumberTraits<DGtal::uint16_t>::max()
  << std::endl;
  trace.info() << "  - min uint16 = " << NumberTraits<DGtal::uint16_t>::min()
  << std::endl;
  trace.info() << "  - max uint32 = " << NumberTraits<DGtal::uint32_t>::max()
  << std::endl;
  trace.info() << "  - min uint32 = " << NumberTraits<DGtal::uint32_t>::min()
  << std::endl;
  trace.info() << "  - max uint64 = " << NumberTraits<DGtal::uint64_t>::max()
  << std::endl;
  trace.info() << "  - min uint64 = " << NumberTraits<DGtal::uint64_t>::min()
  << std::endl;

  trace.endBlock();

  trace.beginBlock ( "Checking CUnsignedInteger models ..." );
  BOOST_CONCEPT_ASSERT(( concepts::CUnsignedNumber<unsigned int> ));
  trace.endBlock();

  trace.beginBlock ( "Checking NumberTraits on  built-in OS dependent types ..." );
  trace.warning() << "If digits() returns 0, the type is not considered in specializations"<<endl;
  trace.info() << "  - digits int = " << NumberTraits<int>::digits()<< std::endl;
  trace.info() << "  - digits unsigned int = " << NumberTraits<unsigned int>::digits()<< std::endl;
  trace.info() << "  - digits long int = " << NumberTraits<long int>::digits()<< std::endl;
  trace.endBlock();
}

TEST_CASE("DGal::BigInteger tests")
{
  trace.beginBlock ( "Checking CUnsignedInteger models ..." );
  BOOST_CONCEPT_ASSERT(( concepts::CInteger<DGtal::BigInteger> ));
  trace.endBlock();

  DGtal::BigInteger big(425);
  CHECK(big == (int)425);

  DGtal::BigInteger anotherbig(5);
  DGtal::BigInteger res = big + anotherbig;
  CHECK(res == (int)430);
}

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
