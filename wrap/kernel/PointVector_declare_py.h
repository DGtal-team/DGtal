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

#include "dgtal_pybind11_common.h"

#include "DGtal/kernel/PointVector.h"

/**
 * Arithmetic operators between types,
 * Note that TOther can be equal to TSelf.
 */
template<typename TOther, typename TSelf>
void declare_PointVector_arithmetic_operators_between_types(pybind11::class_<TSelf> & in_py_class)
{
    namespace py = pybind11;
    in_py_class.def(py::self + TOther());
    in_py_class.def(py::self - TOther());
    in_py_class.def(py::self * TOther());
    in_py_class.def(py::self / TOther());
    in_py_class.def(TOther() + py::self);
    in_py_class.def(TOther() - py::self);
    in_py_class.def(TOther() * py::self);
    in_py_class.def(TOther() / py::self);
}

/**
 * Arithmetic operators (in place)
 */
template<typename TOther, typename TSelf>
void declare_PointVector_arithmetic_in_place_operators_between_types(pybind11::class_<TSelf> & in_py_class)
{
    namespace py = pybind11;
    // in_py_class.def(py::self += TOther());
    in_py_class.def("__iadd__", [](TSelf & self, const TOther & other) {
            return self.operator+=(other);
            });
    // in_py_class.def(py::self -= TOther());
    in_py_class.def("__isub__", [](TSelf & self, const TOther & other) {
            return self.operator-=(other);
            });
    // in_py_class.def(py::self *= TOther());
    in_py_class.def("__imul__", [](TSelf & self, const TOther & other) {
            return self.operator*=(other);
            });
    // in_py_class.def(py::self /= TOther());
    in_py_class.def("__itruediv__", [](TSelf & self, const TOther & other) {
            return self.operator/=(other);
            });
}

/**
 * Comparison operators.
 */
template<typename TOther, typename TSelf>
void declare_PointVector_comparison_operators_between_types(pybind11::class_<TSelf> & in_py_class)
{
    namespace py = pybind11;
    // Comparisons
    in_py_class.def(py::self == TOther());
    in_py_class.def(py::self != TOther());
    in_py_class.def(py::self < TOther());
    in_py_class.def(py::self > TOther());
    in_py_class.def(py::self <= TOther());
    in_py_class.def(py::self >= TOther());
}

/**
 * All member functions that accepts a PointVector with same or different Component type.
 * For example: dotProduct.
 */
template<typename TOther, typename TSelf>
void declare_PointVector_member_functions_between_types(pybind11::class_<TSelf> & in_py_class)
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
void declare_PointVector_all_mixings(pybind11::class_<TSelf> & in_py_class)
{
    declare_PointVector_arithmetic_operators_between_types<TOther>(in_py_class);
    declare_PointVector_comparison_operators_between_types<TOther>(in_py_class);
    declare_PointVector_member_functions_between_types<TOther>(in_py_class);

    declare_PointVector_arithmetic_operators_between_types<typename TOther::Component>(in_py_class);
}

template<typename TPointVector>
pybind11::class_<TPointVector> declare_PointVector(pybind11::module &m,
        const std::string &typestr, const std::string &component_str) {
    namespace py = pybind11;
    using TT = TPointVector;
    using TTComponent = typename TT::Component;

    auto py_class = py::class_<TT>(m, typestr.c_str(), py::buffer_protocol());

    py_class.def_property_readonly_static("dtype",
            [&component_str](py::object /* self */) {
            return component_str;
            });
    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init());
    py_class.def(py::init<const TT &>());
    if(TT::dimension == 2) {
        py_class.def(py::init<const TTComponent &,
                const TTComponent &>());
    } else if(TT::dimension == 3)  {
        py_class.def(py::init<const TTComponent &,
                const TTComponent &,
                const TTComponent &>());
    } else if(TT::dimension == 4)  {
        py_class.def(py::init<const TTComponent &,
                const TTComponent &,
                const TTComponent &,
                const TTComponent &>());
    }
    // ----------------------- Bridges ----------------------------------------
    // Python buffers (requires py::buffer_protocol in py_class instantiation)
    // Allows: numpy.array(an_instance, copy = False)
    py_class.def_buffer([](TT &self) -> py::buffer_info {
        return py::buffer_info(
            self.data(),                                  /* Pointer to buffer */
            static_cast<ssize_t>(sizeof(TTComponent)),    /* Size of one scalar */
            py::format_descriptor<TTComponent>::format(), /* Python struct-style format descriptor */
            1,                                            /* Number of dimensions */
            { TT::dimension },                            /* Shape, buffer dimensions */
            { static_cast<ssize_t>(sizeof(TTComponent)) } /* Strides (in bytes) for each index */
            );
        });

    // Note(phcerdan): A constructor from a py::buffer would be most helpful
    // in a factory/helper function, which will return the appropiate PointVector type
    // i.e PointVector(np.array([2., 3.1, 4.])) will return a RealPoint3D
    py_class.def(py::init([](py::buffer buf) {
        /* Note(phcerdan): Adapted from numpy/stl_bind.h vector_buffer */
        /* Request a buffer descriptor from Python */
        auto info = buf.request();

        /* Sanity checks */
        if (info.ndim != 1 || info.strides[0] % static_cast<ssize_t>(sizeof(TTComponent)))
            throw py::type_error("Only valid 1D buffers can be copied to a PointVector");
        if (!py::detail::compare_buffer_info<TTComponent>::compare(info) || (ssize_t) sizeof(TTComponent) != info.itemsize)
            throw py::type_error("Format mismatch (Python: " + info.format + " C++: " + py::format_descriptor<TTComponent>::format() + ")");

        if(info.shape[0] != TT::dimension)
            throw py::type_error("Shape missmatch (Python: " + std::to_string(info.shape[0]) + " C++: " + std::to_string(TT::dimension) + ")");

        TTComponent *p = static_cast<TTComponent*>(info.ptr);
        return TT(p);
        }));

    // ----------------------- Python operators -------------------------------
    py_class.def("__len__", &TT::size);
    py_class.def("__getitem__", [](const TT & self, const size_t index) {
        if (index >= self.size()) throw py::index_error();
        return self[index];
        });
    py_class.def("__setitem__", [](TT & self, const size_t index,
                const TTComponent value) {
        if (index >= self.size()) throw py::index_error();
        self[index] = value;
        });

    py_class.def("__iter__", [](const TT & self) {
        return py::make_iterator(self.begin(), self.end()); },
        py::keep_alive<0, 1>() /* keep object alive while iterator exists */);

    // ----------------------- Pickling ---------------------------------------
    py_class.def(py::pickle(
            [](const TT & self) { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            using TArray = std::array<TTComponent, TT::dimension >;
            TArray values;
            for(size_t i = 0; i < TT::dimension; ++i){
                values[i] = self[i];
            }
            return py::make_tuple(values);
            },
            [](py::tuple t) { //__setstate__
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
    py_class.def(-py::self);
    // Operators with its own type and its own Component.
    // These functions can be used to add operators between different types.
    declare_PointVector_arithmetic_operators_between_types<TT>(py_class);
    declare_PointVector_arithmetic_in_place_operators_between_types<TT>(py_class);
    declare_PointVector_comparison_operators_between_types<TT>(py_class);

    declare_PointVector_arithmetic_operators_between_types<TTComponent>(py_class);
    declare_PointVector_arithmetic_in_place_operators_between_types<TTComponent>(py_class);


    // ----------------------- Class functions --------------------------------
    // Also use declare_PointVector_member_functions_between_types

    py_class.def("max", &TT::max,
            R"(Returns the maximum component value of a point/vector.)");

    py_class.def("min", &TT::min,
            R"(Returns the minimum component value of a point/vector.)");

    py_class.def("negate", &TT::negate, R"(Negate this vector (in-place).)");

    py_class.def("norm", [](const TT& self) {
            return self.norm();
            },
            R"(Computes the norm of a point/vector. Using the default L_2 norm. Use norm1, or normInfinity for different norms types.)");

    py_class.def("squaredNorm", &TT::squaredNorm,
            R"(Computes the squared norm of a vector as a double.)");

    py_class.def("norm1", &TT::norm1,
            R"(Computes the 1-norm of a vector. The absolute sum of the components.)");

    py_class.def("normInfinity", &TT::normInfinity,
            R"(Computes the infinity-norm of a vector. The maximum absolute value of the components.)");
    py_class.def("getNormalized", &TT::getNormalized,
            R"(Compute the normalization of the vector.
Returns a unitary vector with double as coordinate type.)");

    py_class.def("reset", &TT::reset,
            R"(Resets all the values to zero.)");

    py_class.def("clear", &TT::clear,
            R"(Resets all the values to zero.)");

    py_class.def_static("diagonal", &TT::diagonal,
            R"(Returns the diagonal vector with the input value.
i.e. (val, val, ..., val))");

    py_class.def_static("base", &TT::base,
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
    py_class.def_readonly_static("zero", &TT::zero,
            R"(The zero PointVector.)");
    py_class.def_property_readonly_static("dimension",
            [](py::object /* self */) { return TT::dimension; },
            R"(The dimension of the PointVector.)");

    // ----------------------- Print / Display --------------------------------
    // __str__ is for human readable, pretty-print
    py_class.def("__str__", [](const TT& self) {
            std::stringstream os;
            os << "{";
            for (DGtal::Dimension i = 0; i < TT::dimension ; ++i)
            os << self[ i ] << (i == TT::dimension - 1 ? "" : ", ");
            os << "}";
            return os.str();
            });
    // __repr__ is for reproducibility, shows signature of constructor.
    py_class.def("__repr__", [typestr](const TT& self) {
            std::stringstream os;
            os << typestr << "(";
            for (DGtal::Dimension i = 0; i < TT::dimension ; ++i)
            os << self[ i ] << (i == TT::dimension - 1 ? "" : ", ");
            os << ")";
            return os.str();
            });

    declare_PointVector_member_functions_between_types<TT>(py_class);

    return py_class;
}


#endif
