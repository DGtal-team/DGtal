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

#include "dgtal_nanobind_common.h"

#include "Color_types_py.h" // For Python::Color

namespace nb = nanobind;
using namespace nanobind::literals;
using namespace DGtal;

void init_Color(nb::module_ & m) {
    using Color = Python::Color;
    using TT = Color;
    using TTValue = unsigned char;
    const std::string typestr = "Color";
    auto nb_class = nb::class_<TT>(m, typestr.c_str());

    // ----------------------- Constructors -----------------------------------
    nb_class.def(nb::init());
    nb_class.def(nb::init<const TT &>());
    nb_class.def(nb::init<const unsigned int, TTValue>(),
            "Constructor from int representing a rgb color and an alpha",
            nb::arg("rgb"), nb::arg("alpha")=255);
    nb_class.def(nb::init<const TTValue, const TTValue,
            const TTValue, const TTValue>(),
            "Constructor from red, green, blue and alpha integers.",
            nb::arg("red"), nb::arg("green"), nb::arg("blue"), nb::arg("alpha")=255);
    nb_class.def(nb::init<TTValue, TTValue>(),
            "Constructor from gray scale value [0-255] (and alpha).",
            nb::arg("gray_value"), nb::arg("alpha") = 255);

    // ----------------------- Class operators --------------------------------
    // Comparisons
    nb_class.def(nb::self == nb::self);
    nb_class.def(nb::self != nb::self);
    nb_class.def(nb::self < nb::self);

    // Arithmetic
    nb_class.def(nb::self + nb::self);
    nb_class.def(nb::self - nb::self);
    nb_class.def(nb::self += nb::self);
    nb_class.def(nb::self -= nb::self);

    nb_class.def(nb::self * double());
    nb_class.def(nb::self *= double());

    nb_class.def("__copy__",  [](const TT &self) {
        return TT(self);
    });
    nb_class.def("__deepcopy__", [](const TT &self, nb::dict) {
        return TT(self);
    }, nb::arg("memo"));

    // ----------------------- Class functions --------------------------------
    std::string return_double_docs =
        R"(Return the channel in the [0-1.0] range: float(channel/255.0).)";
    nb_class.def("r", &TT::r, return_double_docs.c_str());
    nb_class.def("g", &TT::g, return_double_docs.c_str());
    nb_class.def("b", &TT::b, return_double_docs.c_str());
    nb_class.def("a", &TT::a, return_double_docs.c_str());

    nb_class.def("getRGB", &TT::getRGB,
            "Return the unsigned integer coding each R, G, B channel on 8 bits, "
            "starting from least significant bit.");
    nb_class.def("getRGBA", &TT::getRGBA,
            "Return the unsigned integer coding each R, G, B, A channel on 8 bits, "
            "starting from least significant bit.");
    nb_class.def("setRGBA", &TT::setRGBA,
            "Set the color using unsigned int coding each channel",
            nb::arg("rgba"));

    nb_class.def("setRGBi", &TT::setRGBi,
            "Set the color using unsigned char [0-255] for each channel",
            nb::arg("red"), nb::arg("green"), nb::arg("blue"), nb::arg("alpha")=255);
    nb_class.def("setRGBf", &TT::setRGBf,
            "Set the color using floats [0-1.0] coding each channel",
            nb::arg("red"), nb::arg("green"), nb::arg("blue"), nb::arg("alpha")=1.0);

    nb_class.def("setFromHSV", &TT::setFromHSV,
               "Set the color using HSV values (floats [0-1.0])",
               nb::arg("hue"), nb::arg("saturation"), nb::arg("value"));
    nb_class.def("getHSV", &TT::getHSV,
                "Return the float HSV values of a color");


    nb_class.def("tikz", &TT::tikz,
            R"(Return a string representation of the color usable in TikZ commands.)");
    nb_class.def("svg", &TT::svg,
            R"(Return a string representation of the color usable in svg commands.)");
    nb_class.def("svgAlpha", &TT::svgAlpha,
            R"(Return a SVG parameter string for the opacity value.)");
    nb_class.def("postscript", &TT::postscript,
            R"(Return a string representation of the color usable in postscript commands.)");
    nb_class.def("valid", &TT::valid,
            R"(Check that color is different than Color.Invalid, i.e (0,0,0,0).)");

    // ----------------------- Class data -------------------------------------

    nb_class.def_property("red",
            nb::overload_cast<>()(&TT::red, nb::const_), // getter
            nb::overload_cast<const TTValue>()(&TT::red) // setter
            );
    nb_class.def_property("green",
            nb::overload_cast<>()(&TT::green, nb::const_), // getter
            nb::overload_cast<const TTValue>()(&TT::green) // setter
            );
    nb_class.def_property("blue",
            nb::overload_cast<>()(&TT::blue, nb::const_), // getter
            nb::overload_cast<const TTValue>()(&TT::blue) // setter
            );
    nb_class.def_property("alpha",
            nb::overload_cast<>()(&TT::alpha, nb::const_), // getter
            nb::overload_cast<const TTValue>()(&TT::alpha) // setter
            );

    // Predefined colors
    // None is a reserved name in Python, rename to Invalid. (0,0,0,0)
    nb_class.def_readonly_static("Invalid", &TT::None);
    nb_class.def_readonly_static("Black", &TT::Black);
    nb_class.def_readonly_static("Gray", &TT::Gray);
    nb_class.def_readonly_static("White", &TT::White);
    nb_class.def_readonly_static("Red", &TT::Red);
    nb_class.def_readonly_static("Green", &TT::Green);
    nb_class.def_readonly_static("Lime", &TT::Lime);
    nb_class.def_readonly_static("Blue", &TT::Blue);
    nb_class.def_readonly_static("Cyan", &TT::Cyan);
    nb_class.def_readonly_static("Magenta", &TT::Magenta);
    nb_class.def_readonly_static("Yellow", &TT::Yellow);
    nb_class.def_readonly_static("Silver", &TT::Silver);
    nb_class.def_readonly_static("Purple", &TT::Purple);
    nb_class.def_readonly_static("Navy", &TT::Navy);
    nb_class.def_readonly_static("Aqua", &TT::Aqua);

    // ----------------------- Print / Display --------------------------------
    nb_class.def("__str__", [](const TT & self) {
        std::stringstream os;
        self.selfDisplay(os);
        return os.str();
    });

    nb_class.def("__repr__", [typestr](const TT & self) {
        std::stringstream os;
        os << typestr;
        os << ": " ;
        self.selfDisplay(os);
        return os.str();
    });

    nb_class.def_buffer([](TT &self) -> nb::buffer_info {
        return nb::buffer_info(
            &self, /* Pointer to buffer */
            static_cast<ssize_t>(sizeof(TTValue)),    /* Size of one scalar */
            nb::format_descriptor<TTValue>::format(), /* Python struct-style format descriptor */
            1,                                            /* Number of dimensions */
            { 4 },                            /* Shape, buffer dimensions */
            { static_cast<ssize_t>(sizeof(TTValue)) } /* Strides (in bytes) for each index */
            );
    });

    nb_class.def(nb::init([](nb::buffer buf) {
        auto info = buf.request();
        /* Sanity checks */
        if (info.ndim != 1 || info.strides[0] % static_cast<ssize_t>(sizeof(TTValue)))
            throw nb::type_error("Only valid 1D buffers can be copied to a Color");
        if (!nb::detail::compare_buffer_info<TTValue>::compare(info) || (ssize_t) sizeof(TTValue) != info.itemsize)
            throw nb::type_error("Format mismatch (Python: " + info.format + " C++: " + nb::format_descriptor<TTValue>::format() + ")");

        if(info.shape[0] != 4)
            throw nb::type_error("Shape missmatch (Python: " + std::to_string(info.shape[0]) + " C++: " + std::to_string(4) + ")");
        TTValue *p = static_cast<TTValue*>(info.ptr);
        return TT(*p, *(p+1), *(p+2), *(p+3));
    }));
}
