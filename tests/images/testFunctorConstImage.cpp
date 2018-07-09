#include <iostream>
#include <cmath>

#include <boost/concept/assert.hpp>

#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/CConstImage.h"
#include "DGtal/images/FunctorConstImage.h"

int main()
{
  const std::size_t N = 2;

  using namespace DGtal;
  using namespace std;

  using real = double;
  using Space = SpaceND<N, int>;
  using Domain = HyperRectDomain<Space>;
  using Point = typename Domain::Point;

  const Domain domain({0,0}, {100,150});

  //auto image = makeFunctorConstImage( domain, [] ( Point p  ) -> real { return std::cos( (p-Point(50,75)).norm()*0.1); } );
  auto image = makeFunctorConstImage( domain, [] ( Point p, Domain const& d  ) -> real { return std::cos( (p-d.lowerBound()).norm()*0.1); } );

  BOOST_CONCEPT_ASSERT(( DGtal::concepts::CConstImage< decltype(image) > ));

  /*
  for ( auto v : image.constRange() )
    std::cout << v << std::endl;
  */

  //DGtal::VTKWriter<Domain>( "testFunctorConstImage", domain ) << "value" << image;

  //std::cout << image(Point(1, 2)) << std::endl;
  std::cout << image(Point(1, 2)) << std::endl;

  return 0;
}
