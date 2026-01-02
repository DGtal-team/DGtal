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


#ifndef DGTAL_POINTVECTOR_DECLARE_PY_H
#define DGTAL_POINTVECTOR_DECLARE_PY_H

#if defined (_MSC_VER) and !defined(ssize_t)
    // ssize_t is not standard, only posix which is not supported by MSVC
    #define ssize_t ptrdiff_t
#endif

#include "dgtal_nanobind_common.h"

#include "DGtal/kernel/PointVector.h"


#ifdef _MSC_VER
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#else
#include <unistd.h>
#endif

/**
 * Arithmetic operators between types,
 * Note that TOther can be equal to TSelf.
 */
template<typename TOther, typename TSelf>
void declare_PointVector_arithmetic_operators_between_types(nanobind::class_<TSelf> & in_py_class)
{
    namespace nb = nanobind;
    using namespace nanobind::literals;
    in_py_class.def(nb::self + TOther());
    in_py_class.def(nb::self - TOther());
    in_py_class.def(nb::self * TOther());
    in_py_class.def(nb::self / TOther());
    in_py_class.def(TOther() + nb::self);
    in_py_class.def(TOther() - nb::self);
    in_py_class.def(TOther() * nb::self);
    in_py_class.def(TOther() / nb::self);
}

/**
 * Arithmetic operators (in place)
 */
template<typename TOther, typename TSelf>
void declare_PointVector_arithmetic_in_place_operators_between_types(nanobind::class_<TSelf> & in_py_class)
{
    namespace nb = nanobind;
    using namespace nanobind::literals;
    // in_py_class.def(nb::self += TOther());
    in_py_class.def("__iadd__", [](TSelf & self, const TOther & other) {
            return self.operator+=(other);
            });
    // in_py_class.def(nb::self -= TOther());
    in_py_class.def("__isub__", [](TSelf & self, const TOther & other) {
            return self.operator-=(other);
            });
    // in_py_class.def(nb::self *= TOther());
    in_py_class.def("__imul__", [](TSelf & self, const TOther & other) {
            return self.operator*=(other);
            });
    // in_py_class.def(nb::self /= TOther());
    in_py_class.def("__itruediv__", [](TSelf & self, const TOther & other) {
            return self.operator/=(other);
            });
}

/**
 * Comparison operators.
 */
template<typename TOther, typename TSelf>
void declare_PointVector_comparison_operators_between_types(nanobind::class_<TSelf> & in_py_class)
{
    namespace nb = nanobind;
    using namespace nanobind::literals;
    // Comparisons
    in_py_class.def(nb::self == TOther());
    in_py_class.def(nb::self != TOther());
    in_py_class.def(nb::self < TOther());
    in_py_class.def(nb::self > TOther());
    in_py_class.def(nb::self <= TOther());
    in_py_class.def(nb::self >= TOther());
}

/**
 * All member functions that accepts a PointVector with same or different Component type.
 * For example: dotProduct.
 */
template<typename TOther, typename TSelf>
void declare_PointVector_member_functions_between_types(nanobind::class_<TSelf> & in_py_class)
{
    const std::string dot_docs =
R"(Dot product with another Point of the same dimension.

Parameters
----------
other : PointND | RealPointND
    Vector of type Integer or Real of same dimension than the object.

Returns
-------
dot: Integer | Real
    The dot product in the best Euclidean ring accordingly to the C++ conversion
    rules in arithmetic operations context.
)";
    in_py_class.def("dot", [](const TSelf & self, const TOther& other) {
            return self.dot(other);
            }, dot_docs.c_str());

    // crossProduct is only compatible with 2D/3D, if 4D is wrapped, this should
    // be moved outside this function, and wrap it only for those types.
    const std::string crossProduct_docs =
R"(Cross product with another PointVector of the same dimension.
Warning Only available in 3D and 2D (return the 3th component of the
corresponding cross produt in 3D).

Parameters
----------
other : PointND | RealPointND
    Vector of type Integer or Real of same dimension than the object.

Returns
-------
cross : PointND | RealPointND
    Cross product of the two vectors
)";
    in_py_class.def("crossProduct", [](const TSelf & self, const TOther& other) {
            return self.crossProduct(other);
            }, crossProduct_docs.c_str());

    const std::string cosineSimilarity_docs =
R"(Provide angle betwen two vectors, deduced from their scalar product.

Parameters
----------
other : PointND | RealPointND
    Vector of type Integer or Real of same dimension than the object.

Returns
-------
angle : Float
    The angle between self and other in [0, pi]
)";
    in_py_class.def("cosineSimilarity", [](const TSelf & self, const TOther& other) {
            return self.cosineSimilarity(other);
            }, cosineSimilarity_docs.c_str());

    const std::string inf_docs =
R"(Infimum (greatest lower bound).
The point whose coordinates are exactly the minimum of the two points,
coordinate by coordinate.

Parameters
----------
other : PointND | RealPointND
    Vector of type Integer or Real of same dimension than the object.

Returns
-------
inf : PointND | RealPointND
    Infimum of the two vectors
)";
    in_py_class.def("inf", [](const TSelf & self, const TOther& other) {
            return self.inf(other);
            }, inf_docs.c_str());

    const std::string sup_docs =
R"(Supremum (least upper bound).
The point whose coordinates are exactly the maximum of the two points,
coordinate by coordinate.

Parameters
----------
other : PointND | RealPointND
    Vector of type Integer or Real of same dimension than the object.

Returns
-------
sup : PointND | RealPointND
    Supremum of the two vectors
)";
    in_py_class.def("sup", [](const TSelf & self, const TOther& other) {
            return self.sup(other);
            }, sup_docs.c_str());

    const std::string isLower_docs =
R"(Return true if this point is below a given point.

Parameters
----------
other : PointND | RealPointND
    Vector of type Integer or Real of same dimension than the object.

Returns
-------
isLower : PointND | RealPointND
    this == inf(this, other), see inf for infimum function.
)";
    in_py_class.def("isLower", [](const TSelf & self, const TOther& other) {
            return self.isLower(other);
            }, isLower_docs.c_str());

    const std::string isUpper_docs =
R"(Return true if this point is upper a given point.

Parameters
----------
other : PointND | RealPointND
    Vector of type Integer or Real of same dimension than the object.

Returns
-------
isUpper : PointND | RealPointND
    this == sup(this, other), see sup for supremum function.
)";
    in_py_class.def("isUpper", [](const TSelf & self, const TOther& other) {
            return self.isUpper(other);
            }, isUpper_docs.c_str());
}


/**
 * Helper function to declare all mixings of a PointVector
 * TOther must be a PointVector type with same dimension than TSelf.
 *
 * Example of usage:
 * ```
 * using Point2D = PointVector<2, DGtal::PythonInteger>;
 * auto py_class_Point2D = declare_PointVector<Point2D>(m,"Point2D");
 * using RealPoint2D = PointVector<2, DGtal::PythonReal>;
 * declare_PointVector_all_mixings<RealPoint2D>(py_class_Point2D);
 * ```
 */
template<typename TOther, typename TSelf>
void declare_PointVector_all_mixings(nanobind::class_<TSelf> & in_py_class)
{
    declare_PointVector_arithmetic_operators_between_types<TOther>(in_py_class);
    declare_PointVector_comparison_operators_between_types<TOther>(in_py_class);
    declare_PointVector_member_functions_between_types<TOther>(in_py_class);

    declare_PointVector_arithmetic_operators_between_types<typename TOther::Component>(in_py_class);
}

template<typename TPointVector>
nanobind::class_<TPointVector> declare_PointVector(nanobind::module_ &m,
        const std::string &typestr, const std::string &component_str) {
    namespace nb = nanobind;
    using namespace nanobind::literals;
    using TT = TPointVector;
    using TTComponent = typename TT::Component;

    const std::string docs =
R"(A PointVector may represent either a symbolic point or a symbolic
vector depending on the context. The coordinates of the point or
the components of the vector should be part of a ring. For
performance reasons, these two types are just aliases. The user
should take care how to use it depending on the context. For
instance, adding two points has no meaning, but will be
authorized by the compiler.

All operations involving PointVector, and some of its methods,
follow the classical arithmetic conversion rules.

Example of usage:

    import dgtal
    Point = dgtal.kernel.RealPoint3D
    point1 = Point(2,4,6)
    point1 = Point(2,4,6)
    # Arithmetic operators
    print(point1 + point1)
    print(point1.norm1())

    # As numpy array (view), no copies.
    import numpy as np
    np_array = np.array(point1, copy = False)
)";
    auto nb_class = nb::class_<TT>(m, typestr.c_str(), docs.c_str());

    nb_class.def_property_readonly_static("dtype",
            [&component_str](nb::object /* self */) {
            return component_str;
            });
    // ----------------------- Constructors -----------------------------------
    nb_class.def(nb::init());
    nb_class.def(nb::init<const TT &>());
    if(TT::dimension == 2) {
        nb_class.def(nb::init<const TTComponent &,
                const TTComponent &>());
    } else if(TT::dimension == 3)  {
        nb_class.def(nb::init<const TTComponent &,
                const TTComponent &,
                const TTComponent &>());
    } else if(TT::dimension == 4)  {
        nb_class.def(nb::init<const TTComponent &,
                const TTComponent &,
                const TTComponent &,
                const TTComponent &>());
    }
    // ----------------------- Bridges ----------------------------------------
    // Python buffers (requires nb::buffer_protocol in nb_class instantiation)
    // Allows: numpy.array(an_instance, copy = False)
    nb_class.def_buffer([](TT &self) -> nb::buffer_info {
        return nb::buffer_info(
            self.data(),                                  /* Pointer to buffer */
            static_cast<ssize_t>(sizeof(TTComponent)),    /* Size of one scalar */
            nb::format_descriptor<TTComponent>::format(), /* Python struct-style format descriptor */
            1,                                            /* Number of dimensions */
            { TT::dimension },                            /* Shape, buffer dimensions */
            { static_cast<ssize_t>(sizeof(TTComponent)) } /* Strides (in bytes) for each index */
            );
        });

    // Note(phcerdan): A constructor from a nb::buffer would be most helpful
    // in a factory/helper function, which will return the appropiate PointVector type
    // i.e PointVector(np.array([2., 3.1, 4.])) will return a RealPoint3D
    /* Buffer init constructor not yet implemented
    nb_class.def(nb::init(...)); */

    // ----------------------- Python operators -------------------------------
    nb_class.def_static("__len__", &TT::size);
    nb_class.def("__getitem__", [](const TT & self, const DGtal::Dimension index) {
        if (index >= self.size()) throw nb::index_error();
        return self[index];
        });
    nb_class.def("__setitem__", [](TT & self, const DGtal::Dimension index,
                const TTComponent value) {
        if (index >= self.size()) throw nb::index_error();
        self[index] = value;
        });

    nb_class.def("__iter__", [](const TT & self) {
        return nb::make_iterator(self.begin(), self.end()); },
        nb::keep_alive<0, 1>() /* keep object alive while iterator exists */);

    // ----------------------- Pickling ---------------------------------------
    nb_class.def(nb::pickle(
            [](const TT & self) { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            using TArray = std::array<TTComponent, TT::dimension >;
            TArray values;
            for(DGtal::Dimension i = 0; i < TT::dimension; ++i){
                values[i] = self[i];
            }
            return nb::make_tuple(values);
            },
            [](nb::tuple t) { //__setstate__
            if(t.size() != 1) {
                throw std::runtime_error("Invalid state!");
            }
            using TArray = std::array<TTComponent, TT::dimension >;
            TT point(t[0].cast<TArray>().data());
            return point;
            }
            ));

    // ----------------------- Class operators --------------------------------
    // Negation
    nb_class.def(-nb::self);
    // Operators with its own type and its own Component.
    // These functions can be used to add operators between different types.
    declare_PointVector_arithmetic_operators_between_types<TT>(nb_class);
    declare_PointVector_arithmetic_in_place_operators_between_types<TT>(nb_class);
    declare_PointVector_comparison_operators_between_types<TT>(nb_class);

    declare_PointVector_arithmetic_operators_between_types<TTComponent>(nb_class);
    declare_PointVector_arithmetic_in_place_operators_between_types<TTComponent>(nb_class);


    // ----------------------- Class functions --------------------------------
    // Also use declare_PointVector_member_functions_between_types

    nb_class.def("max", &TT::max,
            R"(Returns the maximum component value of a point/vector.)");

    nb_class.def("min", &TT::min,
            R"(Returns the minimum component value of a point/vector.)");

    nb_class.def("negate", &TT::negate, R"(Negate this vector (in-place).)");

    nb_class.def("norm", [](const TT& self) {
            return self.norm();
            },
            R"(Computes the norm of a point/vector. Using the default L_2 norm. Use norm1, or normInfinity for different norms types.)");

    nb_class.def("squaredNorm", &TT::squaredNorm,
            R"(Computes the squared norm of a vector as a double.)");

    nb_class.def("norm1", &TT::norm1,
            R"(Computes the 1-norm of a vector. The absolute sum of the components.)");

    nb_class.def("normInfinity", &TT::normInfinity,
            R"(Computes the infinity-norm of a vector. The maximum absolute value of the components.)");
    nb_class.def("getNormalized", &TT::getNormalized,
            R"(Compute the normalization of the vector.
Returns a unitary vector with double as coordinate type.)");

    nb_class.def("reset", &TT::reset,
            R"(Resets all the values to zero.)");

    nb_class.def("clear", &TT::clear,
            R"(Resets all the values to zero.)");

    nb_class.def_static("diagonal", &TT::diagonal,
            R"(Returns the diagonal vector with the input value.
i.e. (val, val, ..., val))");

    nb_class.def_static("base", &TT::base,
            R"(Returns the k-th base vector with the input value.
i.e. Example: the 1-th base vector in 3D: (0, val, 0).

Parameters
----------
k: Int
    Any number between 0 and Dimension - 1
val: Component [Default = 1]
    Vector will have this value at k component.
)");

    // ----------------------- Class data -------------------------------------
    nb_class.def_readonly_static("zero", &TT::zero,
            R"(The zero PointVector.)");
    nb_class.def_property_readonly_static("dimension",
            [](nb::object /* self */) { return TT::dimension; },
            R"(The dimension of the PointVector.)");

    // ----------------------- Print / Display --------------------------------
    // __str__ is for human readable, pretty-print
    nb_class.def("__str__", [](const TT& self) {
            std::stringstream os;
            os << "{";
            for (DGtal::Dimension i = 0; i < TT::dimension ; ++i)
            os << self[ i ] << (i == TT::dimension - 1 ? "" : ", ");
            os << "}";
            return os.str();
            });
    // __repr__ is for reproducibility, shows signature of constructor.
    nb_class.def("__repr__", [typestr](const TT& self) {
            std::stringstream os;
            os << typestr << "(";
            for (DGtal::Dimension i = 0; i < TT::dimension ; ++i)
            os << self[ i ] << (i == TT::dimension - 1 ? "" : ", ");
            os << ")";
            return os.str();
            });

    declare_PointVector_member_functions_between_types<TT>(nb_class);

    return nb_class;
}


#endif
