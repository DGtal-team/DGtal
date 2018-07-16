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
 * @file
 * @ingroup Examples
 * @author Roland Denis (\c denis@math.univ-lyon1.fr )
 * CNRS, Institut Camille Jordan (UMR 5208), Université Lyon 1, France
 *
 * @date 2018/07/19
 *
 * @brief An example file for @ref FunctorHolder and @ref holdFunctor.
 *
 * This file is part of the DGtal library.
 */


//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>

#include "DGtal/base/FunctorHolder.h"
#include "DGtal/kernel/PointVector.h"

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"
#include "DGtal/io/readers/PGMReader.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/kernel/BasicPointPredicates.h"

//////////////////////////////////////////////////////////////////////////////

//! [signed_dist_to_unit_circle]
inline double signed_dist_to_unit_circle(DGtal::PointVector<2, double> const& pt)
{
  return pt.norm() - 1.;
}
//! [signed_dist_to_unit_circle]

//! [Templated signed_dist_to_unit_circle]
template <typename Point>
inline
typename Point::Component
templated_signed_dist_to_unit_circle(Point const& pt)
{
  return pt.norm() - typename Point::Component(1);
}
//! [Templated signed_dist_to_unit_circle]

//! [Functor]
template <typename Point>
struct SignedDistToCircle
{
  using Real = typename Point::Component;

  Point center;
  Real  radius;

  SignedDistToCircle(Point const& pt, Real r)
    : center(pt), radius(r)
  {}

  inline
  Real operator() (Point const& pt) const
    {
      return (pt - center).norm() - radius;
    }
};
//! [Functor]

int main()
{

  /////////////////////////////////////////////////////////////////////////////
  // Holding a function

  {
    std::cout << "Holding a function" << std::endl;
    //! [Holding a function]
    auto fn = DGtal::holdFunctor( signed_dist_to_unit_circle );
    using Point = DGtal::PointVector<2, double>;
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a function]
  }

  {
    std::cout << "Holding a templated function" << std::endl;
    //! [Holding a templated function]
    using Point = DGtal::PointVector<2, double>;
    auto fn = DGtal::holdFunctor( templated_signed_dist_to_unit_circle<Point> );
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a templated function]
  }

  {
    std::cout << "Holding a function through a lambda" << std::endl;
    //! [Holding a function through a lambda]
    using Point = DGtal::PointVector<2, double>;
    auto fn = DGtal::holdFunctor( [] (Point const& pt) { return signed_dist_to_unit_circle(pt); } );
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a function through a lambda]
  }

  {
    std::cout << "Holding a templated function through a lambda" << std::endl;
    //! [Holding a templated function through a lambda]
    using Point = DGtal::PointVector<2, double>;
    auto fn = DGtal::holdFunctor( [] (Point const& pt) { return templated_signed_dist_to_unit_circle(pt); } );
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a templated function through a lambda]
  }

#if __cplusplus >= 201300
  {
    std::cout << "Holding a templated function through a C++14 lambda" << std::endl;
    //! [Holding a templated function through a C++14 lambda]
    auto fn = DGtal::holdFunctor( [] (auto const& pt) { return templated_signed_dist_to_unit_circle(pt); } );
    using Point = DGtal::PointVector<2, double>;
    std::cout << fn( Point(1, 1) ) << std::endl; // <- template parameter is resolved to Point
    //! [Holding a templated function through a C++14 lambda]
  }
#endif

  /////////////////////////////////////////////////////////////////////////////
  // Holding a functor

  {
    std::cout << "Holding a functor by (lvalue) reference" << std::endl;
    //! [Holding a functor by lvalue ref]
    using Point = DGtal::PointVector<2, double>;
    SignedDistToCircle<Point> dist(Point(0, 1), 2);
    auto fn = DGtal::holdFunctor( dist );
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a functor by lvalue ref]
  }

  {
    std::cout << "Holding a functor by rvalue reference" << std::endl;
    //! [Holding a functor by rvalue ref]
    using Point = DGtal::PointVector<2, double>;
    auto fn = DGtal::holdFunctor( SignedDistToCircle<Point>( Point(0, 1), 2 ) );
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a functor by rvalue ref]
  }

  {
    std::cout << "Holding a functor by moving it" << std::endl;
    //! [Holding a functor by moving it]
    using Point = DGtal::PointVector<2, double>;
    SignedDistToCircle<Point> dist(Point(0, 1), 2);
    auto fn = DGtal::holdFunctor( std::move(dist) );
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a functor by moving it]
  }

  /////////////////////////////////////////////////////////////////////////////
  // Holding a lambda

  {
    std::cout << "Holding a lambda" << std::endl;
    //! [Holding a lambda]
    using Point = DGtal::PointVector<2, double>;
    Point center(0, 1);
    double radius = 2;

    auto fn = DGtal::holdFunctor(
        [&center, &radius] (Point const& pt) {
            return (pt - center).norm() - radius;
        }
    );
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a lambda]
  }

  {
    std::cout << "Holding a non-unary lambda" << std::endl;
    //! [Holding a non-unary lambda]
    using Point = DGtal::PointVector<2, double>;

    auto dist = [] (Point const& pt, Point const& center, double radius)
      {
        return (pt - center).norm() - radius;
      };

    auto fn = DGtal::holdFunctor( dist );

    std::cout << fn( Point(1, 1), Point(0, 1), 2 ) << std::endl;
    //! [Holding a non-unary lambda]
  }

  /////////////////////////////////////////////////////////////////////////////
  // Copying

  {
    std::cout << "Copying a functor held by lvalue reference" << std::endl;
    //! [Copying a functor by lvalue ref]
    using Point = DGtal::PointVector<2, double>;
    SignedDistToCircle<Point> dist(Point(0, 1), 2);
    auto fn = DGtal::holdFunctor( dist );
    auto fn2 = fn;

    std::cout << fn( Point(1, 1) ) << std::endl;  // Output: -1
    std::cout << fn2( Point(1, 1) ) << std::endl; // Output: -1

    dist.radius = 3; // fn and fn2 both refer to the same functor dist.

    std::cout << fn( Point(1, 1) ) << std::endl;  // Output: -2
    std::cout << fn2( Point(1, 1) ) << std::endl; // Output: -2
    //! [Copying a functor by lvalue ref]
  }

  {
    std::cout << "Copying a lambda helb by rvalue reference" << std::endl;
    //! [Copying a lambda by rvalue ref]
    int init_cnt = 0;
    auto fn = DGtal::holdFunctor( [init_cnt] () mutable { return ++init_cnt; } );
    std::cout << fn() << std::endl;  // Output: 1
    auto fn2 = fn;
    std::cout << fn2() << std::endl; // Output: 2
    std::cout << fn() << std::endl;  // Output: 3
    //! [Copying a lambda by rvalue ref]
  }
  
  /////////////////////////////////////////////////////////////////////////////
  // Type of FunctorHolder

  {
    //! [Storing a FunctorHolder]
    using Point = DGtal::PointVector<2, double>;
    auto fn = DGtal::holdFunctor( SignedDistToCircle<Point>( Point(0, 1), 2 ) );
    //! [Storing a FunctorHolder]
  }
  
  {
    //image import
    using Image = DGtal::ImageContainerBySTLVector< DGtal::Z2i::Domain, int>;
    using Point = Image::Point;
    std::string filename =  examplesPath + "samples/contourS.pgm";
    Image image = DGtal::PGMReader<Image>::importPGM(filename); 

    //! [Passing a FunctorHolder]
    auto binarizer = DGtal::holdFunctor( [] (Image::Value v) { return v <= 135; } );
    DGtal::functors::PointFunctorPredicate<Image, decltype(binarizer)> predicate(image, binarizer); 
    //! [Passing a FunctorHolder]

    Point const pt(50, 25);
    std::cout << "binarizer(" << image( pt ) << ") = " << predicate( pt ) << std::endl;
  }

  return 0;
}
