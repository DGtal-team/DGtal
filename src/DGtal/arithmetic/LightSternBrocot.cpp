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
 * @file LightSternBrocot.cpp
 * 
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 * @author Xavier Provençal (\c xavier.provencal@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/03/07
 *
 * Implementation of methods defined in LightSternBrocot.h 
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/arithmetic/LightSternBrocot.h"
///////////////////////////////////////////////////////////////////////////////

#include <iostream>

using namespace std;



///////////////////////////////////////////////////////////////////////////////
// class Common
///////////////////////////////////////////////////////////////////////////////

/** DGtal Global variables
*
**/
namespace DGtal
{
  template <typename TInteger, typename TQuotient, typename TMap>
  DGtal::LightSternBrocot<TInteger, TQuotient, TMap>*
  DGtal::LightSternBrocot<TInteger, TQuotient, TMap>::singleton = 0;

  template <>
  LightSternBrocot<DGtal::int32_t,DGtal::int32_t>*
  LightSternBrocot<DGtal::int32_t,DGtal::int32_t>::singleton = 0;

  template <>
  LightSternBrocot<DGtal::int64_t,DGtal::int32_t>*
  LightSternBrocot<DGtal::int64_t,DGtal::int32_t>::singleton = 0;

  template <>
  LightSternBrocot<DGtal::int64_t,DGtal::int64_t>*
  LightSternBrocot<DGtal::int64_t,DGtal::int64_t>::singleton = 0;

#ifdef WITH_BIGINTEGER
  template <>
  LightSternBrocot<DGtal::BigInteger,DGtal::int32_t>*
  LightSternBrocot<DGtal::BigInteger,DGtal::int32_t>::singleton = 0;

  template <>
  LightSternBrocot<DGtal::BigInteger,DGtal::int64_t>*
  LightSternBrocot<DGtal::BigInteger,DGtal::int64_t>::singleton = 0;

  template <>
  LightSternBrocot<DGtal::BigInteger,DGtal::BigInteger>*
  LightSternBrocot<DGtal::BigInteger,DGtal::BigInteger>::singleton = 0;

#endif

}
