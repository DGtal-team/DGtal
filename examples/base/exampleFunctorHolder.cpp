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
 * @date 2018/07/17
 *
 * @brief An example file for @ref DGtal::functors::FunctorHolder and @ref DGtal::functors::holdFunctor.
 *
 * This file is part of the DGtal library.
 */


//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <numeric>
#include <iterator>
#include <utility>

#include "DGtal/base/FunctorHolder.h"
#include "DGtal/kernel/PointVector.h"

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"
#include "DGtal/io/readers/PGMReader.h"
#include "DGtal/io/readers/GenericReader.h"
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

//! [Returning a FunctorHolder]
template <typename T>
struct Binarizer
{
  T threshold;
  explicit Binarizer(T v) : threshold(v) {}
  Binarizer& operator= (Binarizer const&) = delete; // This is not a model of boost::Assignable
  bool operator() (T v) const { return v <= threshold; }
};

template <typename T>
inline
decltype(DGtal::functors::holdFunctor(Binarizer<T>(128))) // Deduced return type
get_trivial_binarizer()
{
  return DGtal::functors::holdFunctor( Binarizer<T>(128) );
}
//! [Returning a FunctorHolder]

//! [Returning a FunctorHolder using trailing return]
template <typename Iterator>
auto get_mean_binarizer_from_range(Iterator first, Iterator last) // auto as return type
     -> decltype(DGtal::functors::holdFunctor(Binarizer<decltype(*first / std::distance(first, last))>(0)))
     // with trailing return type specification using ->
{
  using value_type = typename std::iterator_traits<Iterator>::value_type;
  auto const mean = std::accumulate(first, last, value_type(0));
  auto const size = std::distance(first, last);
  return DGtal::functors::holdFunctor(Binarizer<decltype(mean / size)>(mean / size));
}
//! [Returning a FunctorHolder using trailing return]

//! [Returning a FunctorHolder using trailing return and declval]
template <typename Image>
auto get_mean_binarizer_from_an_image(std::string const& file_name)
    -> decltype(get_mean_binarizer_from_range(
          std::declval<Image>().begin(), std::declval<Image>().end() ))
{
  Image const image = DGtal::GenericReader<Image>::import(file_name);
  return get_mean_binarizer_from_range(image.begin(), image.end());
}
//! [Returning a FunctorHolder using trailing return and declval]

#if __cplusplus >= 201402L
//! [Returning a FunctorHolder using auto in C++14]
template <typename Iterator>
auto get_mean_binarizer_from_range_cpp14(Iterator first, Iterator last)
{
  using value_type = typename std::iterator_traits<Iterator>::value_type;
  auto const mean = std::accumulate(first, last, value_type(0));
  auto const size = std::distance(first, last);
  return DGtal::functors::holdFunctor(Binarizer<decltype(mean / size)>(mean / size));
}
//! [Returning a FunctorHolder using auto in C++14]
#endif

//! [Factory of Binarizer]
template <typename T>
inline
Binarizer<T> makeBinarizer( T const& v ) // T auto-deduced from the parameter v
{
  return Binarizer<T>(v);
}
//! [Factory of Binarizer]

#if 0
//! [Wrong factory of PointFunctorPredicate]
template <
  typename PointFunctor,
  typename Predicate
>
DGtal::functors::PointFunctorPredicate<PointFunctor, Predicate>
makePointFunctorPredicate_Example(
    DGtal::ConstAlias<PointFunctor> aFun,
    DGtal::ConstAlias<Predicate> aPred)
{
  return DGtal::functors::PointFunctorPredicate<PointFunctor, Predicate>( aFun, aPred );
}
//! [Wrong factory of PointFunctorPredicate]
#endif

//! [Factory of PointFunctorPredicate]
template <
  typename PointFunctor,
  typename Predicate
>
DGtal::functors::PointFunctorPredicate<PointFunctor, Predicate>
makePointFunctorPredicate_Example( PointFunctor const& aFun, Predicate const& aPred )
{
  return DGtal::functors::PointFunctorPredicate<PointFunctor, Predicate>( aFun, aPred );
}
//! [Factory of PointFunctorPredicate]

//! [Factory of PointFunctorPredicate using perfect forwarding]
template <
  typename PointFunctor,
  typename Predicate
>
DGtal::functors::PointFunctorPredicate<
  typename std::decay<PointFunctor>::type,
  typename std::decay<Predicate>::type
>
makePointFunctorPredicate_Example2( PointFunctor && aFun, Predicate && aPred )
{
  return DGtal::functors::PointFunctorPredicate<
      typename std::decay<PointFunctor>::type,
      typename std::decay<Predicate>::type
  >(
    std::forward<PointFunctor>(aFun),
    std::forward<Predicate>(aPred)
  );
}
//! [Factory of PointFunctorPredicate using perfect forwarding]

///////////////////////////////////////////////////////////////////////////////
int main()
{

  /////////////////////////////////////////////////////////////////////////////
  // Holding a function

  {
    std::cout << "Holding a function" << std::endl;
    //! [Holding a function]
    auto fn = DGtal::functors::holdFunctor( signed_dist_to_unit_circle );
    using Point = DGtal::PointVector<2, double>;
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a function]
  }

  {
    std::cout << "Holding a templated function" << std::endl;
    //! [Holding a templated function]
    using Point = DGtal::PointVector<2, double>;
    auto fn = DGtal::functors::holdFunctor( templated_signed_dist_to_unit_circle<Point> );
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a templated function]
  }

  {
    std::cout << "Holding a function through a lambda" << std::endl;
    //! [Holding a function through a lambda]
    using Point = DGtal::PointVector<2, double>;
    auto fn = DGtal::functors::holdFunctor( [] (Point const& pt) { return signed_dist_to_unit_circle(pt); } );
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a function through a lambda]
  }

  {
    std::cout << "Holding a templated function through a lambda" << std::endl;
    //! [Holding a templated function through a lambda]
    using Point = DGtal::PointVector<2, double>;
    auto fn = DGtal::functors::holdFunctor( [] (Point const& pt) { return templated_signed_dist_to_unit_circle(pt); } );
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a templated function through a lambda]
  }

#if __cplusplus >= 201402L
  {
    std::cout << "Holding a templated function through a C++14 lambda" << std::endl;
    //! [Holding a templated function through a C++14 lambda]
    auto fn = DGtal::functors::holdFunctor( [] (auto const& pt) { return templated_signed_dist_to_unit_circle(pt); } );
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
    auto fn = DGtal::functors::holdFunctor( dist );
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a functor by lvalue ref]
  }

  {
    std::cout << "Holding a functor by rvalue reference" << std::endl;
    //! [Holding a functor by rvalue ref]
    using Point = DGtal::PointVector<2, double>;
    auto fn = DGtal::functors::holdFunctor( SignedDistToCircle<Point>( Point(0, 1), 2 ) );
    std::cout << fn( Point(1, 1) ) << std::endl;
    //! [Holding a functor by rvalue ref]
  }

  {
    std::cout << "Holding a functor by moving it" << std::endl;
    //! [Holding a functor by moving it]
    using Point = DGtal::PointVector<2, double>;
    SignedDistToCircle<Point> dist(Point(0, 1), 2);
    auto fn = DGtal::functors::holdFunctor( std::move(dist) );
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

    auto fn = DGtal::functors::holdFunctor(
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

    auto fn = DGtal::functors::holdFunctor( dist );

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
    auto fn = DGtal::functors::holdFunctor( dist );
    auto fn2 = fn;

    std::cout << fn( Point(1, 1) ) << std::endl;  // Output: -1
    std::cout << fn2( Point(1, 1) ) << std::endl; // Output: -1

    dist.radius = 3; // fn and fn2 both refer to the same functor dist.

    std::cout << fn( Point(1, 1) ) << std::endl;  // Output: -2
    std::cout << fn2( Point(1, 1) ) << std::endl; // Output: -2
    //! [Copying a functor by lvalue ref]
  }

  {
    std::cout << "Copying a lambda held by rvalue reference" << std::endl;
    //! [Copying a lambda by rvalue ref]
    int init_cnt = 0;
    auto fn = DGtal::functors::holdFunctor( [init_cnt] () mutable { return ++init_cnt; } );
    std::cout << fn() << std::endl;  // Output: 1
    auto fn2 = fn;
    std::cout << fn2() << std::endl; // Output: 2
    std::cout << fn() << std::endl;  // Output: 3
    //! [Copying a lambda by rvalue ref]
  }

  /////////////////////////////////////////////////////////////////////////////
  // Type of FunctorHolder

  {
    std::cout << "Storing a FunctorHolder" << std::endl;

    //! [Storing a FunctorHolder]
    using Point = DGtal::PointVector<2, double>;
    auto fn = DGtal::functors::holdFunctor( SignedDistToCircle<Point>( Point(0, 1), 2 ) );
    //! [Storing a FunctorHolder]
  }

  {
    std::cout << "Passing a FunctorHolder" << std::endl;

    //image import
    using Image = DGtal::ImageContainerBySTLVector< DGtal::Z2i::Domain, int>;
    using Point = Image::Point;
    std::string filename =  examplesPath + "samples/contourS.pgm";
    Image image = DGtal::PGMReader<Image>::importPGM(filename);

    //! [Passing a FunctorHolder]
    auto binarizer = DGtal::functors::holdFunctor( [] (Image::Value v) { return v <= 135; } );
    DGtal::functors::PointFunctorPredicate<Image, decltype(binarizer)> predicate(image, binarizer);
    //! [Passing a FunctorHolder]

    Point const pt(50, 25);
    std::cout << "binarizer(" << image( pt ) << ") = " << predicate( pt ) << std::endl;
  }

  {
    std::cout << "Returning a functor" << std::endl;
    //! [Returning a FunctorHolder in caller]
    auto binarizer = get_trivial_binarizer<int>();
    //! [Returning a FunctorHolder in caller]
    std::cout << "binarizer(120) = " << binarizer(120) << std::endl;
  }

  {
    std::cout << "Returning a functor using trailing return syntax" << std::endl;

    //image import
    using Image = DGtal::ImageContainerBySTLVector< DGtal::Z2i::Domain, int>;
    using Point = Image::Point;
    std::string filename =  examplesPath + "samples/contourS.pgm";
    Image image = DGtal::PGMReader<Image>::importPGM(filename);

    //! [Returning a FunctorHolder using trailing return in caller]
    auto binarizer = get_mean_binarizer_from_range(image.begin(), image.end());
    DGtal::functors::PointFunctorPredicate<Image, decltype(binarizer)> predicate(image, binarizer);
    //! [Returning a FunctorHolder using trailing return in caller]

    Point const pt(50, 25);
    std::cout << "binarizer(" << image( pt ) << ") = " << predicate( pt ) << std::endl;
  }

  {
    std::cout << "Returning a functor using trailing return syntax and declval" << std::endl;

    //image import
    using Image = DGtal::ImageContainerBySTLVector< DGtal::Z2i::Domain, int>;
    std::string filename =  examplesPath + "samples/contourS.pgm";

    auto binarizer = get_mean_binarizer_from_an_image<Image>(filename);

    std::cout << "binarizer(120) = " << binarizer(120) << std::endl;
  }

  #if __cplusplus >= 201402L
  {
    std::cout << "Returning a functor using auto in C++14" << std::endl;

    //image import
    using Image = DGtal::ImageContainerBySTLVector< DGtal::Z2i::Domain, int>;
    using Point = Image::Point;
    std::string filename =  examplesPath + "samples/contourS.pgm";
    Image image = DGtal::PGMReader<Image>::importPGM(filename);

    auto binarizer = get_mean_binarizer_from_range_cpp14(image.begin(), image.end());
    DGtal::functors::PointFunctorPredicate<Image, decltype(binarizer)> predicate(image, binarizer);

    Point const pt(50, 25);
    std::cout << "binarizer(" << image( pt ) << ") = " << predicate( pt ) << std::endl;
  }
  #endif

  {
    std::cout << "Usage of Binarizer factory" << std::endl;

    //image import
    using Image = DGtal::ImageContainerBySTLVector< DGtal::Z2i::Domain, int>;
    using Point = Image::Point;
    std::string filename =  examplesPath + "samples/contourS.pgm";
    Image image = DGtal::PGMReader<Image>::importPGM(filename);

    //! [Using the Binarizer factory]
    auto binarizer = DGtal::functors::holdFunctor( makeBinarizer(135) );
    DGtal::functors::PointFunctorPredicate<Image, decltype(binarizer)> predicate(image, binarizer);
    //! [Using the Binarizer factory]

    Point const pt(50, 25);
    std::cout << "binarizer(" << image( pt ) << ") = " << predicate( pt ) << std::endl;
  }


  #if __cplusplus >= 201703L
  {
    std::cout << "Binarizer factory using deduction guides of C++17" << std::endl;

    //image import
    using Image = DGtal::ImageContainerBySTLVector< DGtal::Z2i::Domain, int>;
    using Point = Image::Point;
    std::string filename =  examplesPath + "samples/contourS.pgm";
    Image image = DGtal::PGMReader<Image>::importPGM(filename);

    //! [Binarizer deduction guide in C++17]
    auto binarizer = DGtal::functors::holdFunctor( Binarizer(135) ); // Binarizer template parameter is not specified.
    DGtal::functors::PointFunctorPredicate<Image, decltype(binarizer)> predicate(image, binarizer);
    //! [Binarizer deduction guide in C++17]

    Point const pt(50, 25);
    std::cout << "binarizer(" << image( pt ) << ") = " << predicate( pt ) << std::endl;
  }
  #endif

  {
    std::cout << "Usage of PointFunctorPredicate factory" << std::endl;

    //image import
    using Image = DGtal::ImageContainerBySTLVector< DGtal::Z2i::Domain, int>;
    using Point = Image::Point;
    std::string filename =  examplesPath + "samples/contourS.pgm";
    Image image = DGtal::PGMReader<Image>::importPGM(filename);

    //! [Using the PointFunctorPredicate factory]
    auto binarizer = DGtal::functors::holdFunctor( makeBinarizer(135) );
    auto predicate = makePointFunctorPredicate_Example( image, binarizer );
    //! [Using the PointFunctorPredicate factory]

    Point const pt(50, 25);
    std::cout << "binarizer(" << image( pt ) << ") = " << predicate( pt ) << std::endl;
  }

  {
    std::cout << "Usage of PointFunctorPredicate factory with perfect forwarding" << std::endl;

    //image import
    using Image = DGtal::ImageContainerBySTLVector< DGtal::Z2i::Domain, int>;
    using Point = Image::Point;
    std::string filename =  examplesPath + "samples/contourS.pgm";
    Image image = DGtal::PGMReader<Image>::importPGM(filename);

    auto binarizer = DGtal::functors::holdFunctor( makeBinarizer(135) );
    auto predicate = makePointFunctorPredicate_Example2( image, binarizer );

    Point const pt(50, 25);
    std::cout << "binarizer(" << image( pt ) << ") = " << predicate( pt ) << std::endl;
  }
  return 0;
}
