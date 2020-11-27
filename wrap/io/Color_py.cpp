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

#include "dgtal_pybind11_common.h"

#include "Color_types_py.h" // For Python::Color

namespace py = pybind11;
using namespace DGtal;

void init_Color(py::module & m) {
    using Color = Python::Color;
    using TT = Color;
    using TTValue = unsigned char;
    const std::string typestr = "Color";
    auto py_class = py::class_<TT>(m, typestr.c_str());

    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init());
    py_class.def(py::init<const TT &>());
    py_class.def(py::init<const unsigned int, TTValue>(),
            "Constructor from int representing a rgb color and an alpha",
            py::arg("rgb"), py::arg("alpha")=255);
    py_class.def(py::init<const TTValue, const TTValue,
            const TTValue, const TTValue>(),
            "Constructor from red, green, blue and alpha integers.",
            py::arg("red"), py::arg("green"), py::arg("blue"), py::arg("alpha")=255);
    py_class.def(py::init<TTValue, TTValue>(),
            "Constructor from gray scale value [0-255] (and alpha).",
            py::arg("gray_value"), py::arg("alpha") = 255);

    // ----------------------- Class operators --------------------------------
    // Comparisons
    py_class.def(py::self == py::self);
    py_class.def(py::self != py::self);
    py_class.def(py::self < py::self);

    // Arithmetic
    py_class.def(py::self + py::self);
    py_class.def(py::self - py::self);
    py_class.def(py::self += py::self);
    py_class.def(py::self -= py::self);

    py_class.def(py::self * double());
    py_class.def(py::self *= double());

    py_class.def("__copy__",  [](const TT &self) {
        return TT(self);
    });
    py_class.def("__deepcopy__", [](const TT &self, py::dict) {
        return TT(self);
    }, py::arg("memo"));

    // ----------------------- Class functions --------------------------------
    std::string return_double_docs =
        R"(Return the channel in the [0-1.0] range: float(channel/255.0).)";
    py_class.def("r", &TT::r, return_double_docs.c_str());
    py_class.def("g", &TT::g, return_double_docs.c_str());
    py_class.def("b", &TT::b, return_double_docs.c_str());
    py_class.def("a", &TT::a, return_double_docs.c_str());

    py_class.def("getRGB", &TT::getRGB,
            "Return the unsigned integer coding each R, G, B channel on 8 bits, "
            "starting from least significant bit.");
    py_class.def("getRGBA", &TT::getRGBA,
            "Return the unsigned integer coding each R, G, B, A channel on 8 bits, "
            "starting from least significant bit.");
    py_class.def("setRGBA", &TT::setRGBA,
            "Set the color using unsigned int coding each channel",
            py::arg("rgba"));

    py_class.def("setRGBi", &TT::setRGBi,
            "Set the color using unsigned char [0-255] for each channel",
            py::arg("red"), py::arg("green"), py::arg("blue"), py::arg("alpha")=255);
    py_class.def("setRGBf", &TT::setRGBf,
            "Set the color using floats [0-1.0] coding each channel",
            py::arg("red"), py::arg("green"), py::arg("blue"), py::arg("alpha")=1.0);


    py_class.def("tikz", &TT::tikz,
            R"(Return a string representation of the color usable in TikZ commands.)");
    py_class.def("svg", &TT::svg,
            R"(Return a string representation of the color usable in svg commands.)");
    py_class.def("svgAlpha", &TT::svgAlpha,
            R"(Return a SVG parameter string for the opacity value.)");
    py_class.def("postscript", &TT::postscript,
            R"(Return a string representation of the color usable in postscript commands.)");
    py_class.def("valid", &TT::valid,
            R"(Check that color is different than Color.Invalid, i.e (0,0,0,0).)");

    // ----------------------- Class data -------------------------------------

    py_class.def_property("red",
            py::detail::overload_cast_impl<>()(&TT::red, py::const_), // getter
            py::detail::overload_cast_impl<const TTValue>()(&TT::red) // setter
            );
    py_class.def_property("green",
            py::detail::overload_cast_impl<>()(&TT::green, py::const_), // getter
            py::detail::overload_cast_impl<const TTValue>()(&TT::green) // setter
            );
    py_class.def_property("blue",
            py::detail::overload_cast_impl<>()(&TT::blue, py::const_), // getter
            py::detail::overload_cast_impl<const TTValue>()(&TT::blue) // setter
            );
    py_class.def_property("alpha",
            py::detail::overload_cast_impl<>()(&TT::alpha, py::const_), // getter
            py::detail::overload_cast_impl<const TTValue>()(&TT::alpha) // setter
            );

    // Predefined colors
    // None is a reserved name in Python, rename to Invalid. (0,0,0,0)
    py_class.def_readonly_static("Invalid", &TT::None);
    py_class.def_readonly_static("Black", &TT::Black);
    py_class.def_readonly_static("Gray", &TT::Gray);
    py_class.def_readonly_static("White", &TT::White);
    py_class.def_readonly_static("Red", &TT::Red);
    py_class.def_readonly_static("Green", &TT::Green);
    py_class.def_readonly_static("Lime", &TT::Lime);
    py_class.def_readonly_static("Blue", &TT::Blue);
    py_class.def_readonly_static("Cyan", &TT::Cyan);
    py_class.def_readonly_static("Magenta", &TT::Magenta);
    py_class.def_readonly_static("Yellow", &TT::Yellow);
    py_class.def_readonly_static("Silver", &TT::Silver);
    py_class.def_readonly_static("Purple", &TT::Purple);
    py_class.def_readonly_static("Navy", &TT::Navy);
    py_class.def_readonly_static("Aqua", &TT::Aqua);

    // ----------------------- Print / Display --------------------------------
    py_class.def("__str__", [](const TT & self) {
        std::stringstream os;
        self.selfDisplay(os);
        return os.str();
    });

    py_class.def("__repr__", [typestr](const TT & self) {
        std::stringstream os;
        os << typestr;
        os << ": " ;
        self.selfDisplay(os);
        return os.str();
    });
}
