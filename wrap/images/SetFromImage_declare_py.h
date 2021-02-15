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

#ifndef DGTAL_SETFROMIMAGE_DECLARE_PY_H
#define DGTAL_SETFROMIMAGE_DECLARE_PY_H

#include "dgtal_pybind11_common.h"

#include "DGtal/images/imagesSetsUtils/SetFromImage.h"

template<typename TImageContainer, typename TDigitalSet, typename TPyClassImageContainer >
void add_set_from_image( TPyClassImageContainer & py_class) {
    namespace py = pybind11;
    using TImageValue = typename TImageContainer::Value;

    py_class.def("set_from_image", [](
                const TImageContainer & self,
                const TImageValue & min_value,
                const TImageValue & max_value) {
        TDigitalSet digital_set(self.domain());
        DGtal::SetFromImage<TDigitalSet>::append(digital_set,
                self, min_value, max_value);
        return digital_set;
    },
R"(Create a digital set from this image.

This method will construct a default ForegroundPredicate
instance as a simple thresholding (SimpleForegroundPredicate)
of values in ]min_value,max_value].

Parameters
----------
min_value: ImageContainer::Value
    minimum value of the thresholding
max_value: ImageContainer::Value
    maximum value of the thresholding

Return
------
A DigitalSet with all the values from the image between ]min_value and max_value]
)", py::arg("min_value"), py::arg("max_value")
    );
}
#endif
