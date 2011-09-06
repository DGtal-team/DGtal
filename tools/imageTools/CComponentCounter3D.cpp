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
 * @file CComponentCounter3D.cpp
 * @ingroup tools
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr)
 * LIRIS (CNRS, UMR 5205), University de Lyon, France.
 *
 * @date 2011/05/04
 *
 * A simple program to count the number of connected components in a 3D image..
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/io/Color.h"
#include "DGtal/images/ImageSelector.h"

#include <boost/pending/disjoint_sets.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
namespace po = boost::program_options;



template <typename Rank, typename Parent, typename Image>
void CCCounter(Rank& r, Parent& p, const Image& elements, const unsigned int connectivity)
{
  
  boost::disjoint_sets<Rank,Parent> dsets(r, p);
  trace.beginBlock("Initial disjoint sets construction");
  for(typename Image::Domain::ConstIterator e = elements.domain().begin();
      e != elements.domain().end(); ++e)
    dsets.make_set(*e);
  trace.endBlock();

  trace.beginBlock("Merging neighboring sets");
  typename Image::Point decx(1,0,0);
  typename Image::Point decy(0,1,0);
  typename Image::Point decz(0,0,1);
  
  //Merging process
  for(typename Image::Domain::ConstIterator e = elements.domain().begin();
      e !=elements.domain().end(); ++e)
    {
      if ( elements.domain().isInside(*e+decx) &&  
     (elements(*e) == elements(*e+decx)))
   dsets.union_set(*e,*e+decx);
      
      if ( elements.domain().isInside(*e+decy) &&  
     (elements(*e) == elements(*e+decy)))
  dsets.union_set(*e,*e+decy);
      
      if ( elements.domain().isInside(*e+decz) &&  
     (elements(*e) == elements(*e+decz)))
  dsets.union_set(*e,*e+decz);

      if (connectivity > 6)
  {
    if ( elements.domain().isInside(*e+decx+decy) &&  
         (elements(*e) == elements(*e+decx+decy)))
      dsets.union_set(*e,*e+decx+decy);
      
    if ( elements.domain().isInside(*e+decx+decz) &&  
         (elements(*e) == elements(*e+decx+decz)))
      dsets.union_set(*e,*e+decx+decz);
  
    if ( elements.domain().isInside(*e+decy+decz) &&  
         (elements(*e) == elements(*e+decy+decz)))
      dsets.union_set(*e,*e+decy+decz);
  
    if (connectivity == 26)
      if ( elements.domain().isInside(*e+decy+decz+decx) &&  
     (elements(*e) == elements(*e+decy+decz+decx)))
        dsets.union_set(*e,*e+decy+decz+decx);
    
  }
      
    }
  trace.endBlock();
  std::cout << "Number of disjoint "<<connectivity<<"-components = "
         <<dsets.count_sets(elements.domain().begin(),
          elements.domain().end())
         << std::endl;
}



int main( int argc, char** argv )
{
  // parse command line ----------------------------------------------
  po::options_description general_opt("Allowed options are");
  general_opt.add_options()
    ("help,h", "display this message")
    ("connectivity,c", po::value<unsigned int>()->default_value(6), "object connectivity (6,18,26)"    " (default: 6 )")
    ("input-file,i", po::value<std::string>(), "volume file (Vol)"    " (default: standard input)");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if(vm.count("help")||argc<=1)
    {
      std::cout << "Usage: " << argv[0] << " [input-file]\n"
    << "Count the number of connected component (same values) in a  volume (Vol) file image\n"
    << general_opt << "\n";
      return 0;
    }
 string inputFilename = vm["input-file"].as<std::string>();
 unsigned int connectivity = vm["connectivity"].as<unsigned int>();
 
 if ((connectivity != 6) && (connectivity != 18) && (connectivity != 26))
   {
     trace.error() << "Bard connectivity value.";
     trace.info() << std::endl;
     exit(1);
   }

 typedef ImageSelector<Domain, unsigned char>::Type Image;
 Image image = VolReader<Image>::importVol( inputFilename );

 trace.info() << "Image loaded: "<<image<< std::endl;

 typedef std::map<Point,std::size_t> rank_t; // => order on Element
 typedef std::map<Point,Point> parent_t;
 rank_t rank_map;
 parent_t parent_map;
 
 boost::associative_property_map<rank_t>   rank_pmap(rank_map);
 boost::associative_property_map<parent_t> parent_pmap(parent_map);
 
 CCCounter(rank_pmap, parent_pmap, image, connectivity);
 

 return 0;
}
