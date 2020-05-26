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

#include <DGtal/topology/NeighborhoodConfigurations.h>
#include <DGtal/topology/tables/NeighborhoodLUT.h>
#include <DGtal/topology/tables/NeighborhoodTables.h>

DGtal::CountedPtr< boost::dynamic_bitset<> >
DGtal::simplicity::LUTSimple26_6 =
DGtal::functions::loadTable(DGtal::simplicity::tableSimple26_6);

DGtal::CountedPtr< boost::dynamic_bitset<> >
DGtal::simplicity::LUTSimple6_26 =
DGtal::functions::loadTable(DGtal::simplicity::tableSimple6_26);

DGtal::CountedPtr< boost::dynamic_bitset<> >
DGtal::simplicity::LUTSimple18_6 =
DGtal::functions::loadTable(DGtal::simplicity::tableSimple18_6);

DGtal::CountedPtr< boost::dynamic_bitset<> >
DGtal::simplicity::LUTSimple6_18 =
DGtal::functions::loadTable(DGtal::simplicity::tableSimple6_18);

DGtal::CountedPtr< boost::dynamic_bitset<> >
DGtal::simplicity::LUTSimple8_4 =
DGtal::functions::loadTable<2>(DGtal::simplicity::tableSimple8_4);

DGtal::CountedPtr< boost::dynamic_bitset<> >
DGtal::simplicity::LUTSimple4_8 =
DGtal::functions::loadTable<2>(DGtal::simplicity::tableSimple4_8);

DGtal::CountedPtr< boost::dynamic_bitset<> >
DGtal::isthmusicity::LUTIsthmus =
DGtal::functions::loadTable(DGtal::isthmusicity::tableIsthmus);

DGtal::CountedPtr< boost::dynamic_bitset<> >
DGtal::isthmusicity::LUTOneIsthmus =
DGtal::functions::loadTable(DGtal::isthmusicity::tableOneIsthmus);

DGtal::CountedPtr< boost::dynamic_bitset<> >
DGtal::isthmusicity::LUTTwoIsthmus =
DGtal::functions::loadTable(DGtal::isthmusicity::tableTwoIsthmus);
