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
namespace nb = nanobind;

void init_dgtal_kernel(nb::module_ &);
void init_dgtal_base(nb::module_ &);
void init_dgtal_topology(nb::module_ &);
void init_dgtal_images(nb::module_ &);
void init_dgtal_io(nb::module_ &);
void init_dgtal_helpers(nb::module_&);

NB_MODULE(_dgtal, m) {
    m.doc() = "Digital Geometry Tools and Algorithms.";
    init_dgtal_kernel(m);
    init_dgtal_base(m);
    init_dgtal_topology(m);
    init_dgtal_images(m);
    init_dgtal_io(m);
    init_dgtal_helpers(m);
}
