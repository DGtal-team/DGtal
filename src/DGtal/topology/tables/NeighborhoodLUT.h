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
* @file   NeighborhoodLUT.h
* @author Pablo Hernandez-Cerdan
* @date 2020/05/26
*
* Header for DGtalLUT library.

* You can use setTable(DGtal::simplicity:LUTSimple26_6)
* @see NeighborhoodConfigurations.h
*
**/

#include <DGtal/base/CountedPtr.h>
#include "boost/dynamic_bitset.hpp"

namespace DGtal {
  namespace simplicity  {
    extern DGtal::CountedPtr< boost::dynamic_bitset<> > LUTSimple26_6;
    extern DGtal::CountedPtr< boost::dynamic_bitset<> > LUTSimple6_26;
    extern DGtal::CountedPtr< boost::dynamic_bitset<> > LUTSimple18_6;
    extern DGtal::CountedPtr< boost::dynamic_bitset<> > LUTSimple6_18;
    extern DGtal::CountedPtr< boost::dynamic_bitset<> > LUTSimple8_4;
    extern DGtal::CountedPtr< boost::dynamic_bitset<> > LUTSimple4_8;
  } // simplicity namespace
  namespace isthmusicity {
    extern DGtal::CountedPtr< boost::dynamic_bitset<> > LUTIsthmus;
    extern DGtal::CountedPtr< boost::dynamic_bitset<> > LUTOneIsthmus;
    extern DGtal::CountedPtr< boost::dynamic_bitset<> > LUTTwoIsthmus;
  } // isthmusicity namespace
} // DGtal namespace
