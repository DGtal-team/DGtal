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
 * @file testDomainSplitter.cpp
 * @ingroup Tests
 * @author David Coeurjolly  (\c david.coeurjolly@cnrs.fr)
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 *
 * @date 2026/04/03
 *
 * Functions for testing class DomainSplitter
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtalCatch.h"

#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/domains/DomainSplitter.h"

#ifdef DGTAL_TESTS_WITH_VIEWER
#ifdef DGTAL_WITH_POLYSCOPE_VIEWER
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/viewers/PolyscopeViewer.h"
#endif
#endif

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;
using namespace Z3i;

TEST_CASE( "Domain Regular Grid Splitter tests" )
{
    Domain domain(Point(0,0,0), Point(16,32,64));

    RegularDomainSplitter<Domain> splitter;

    RegularDomainSplitter<Domain>::SplitDomainsInfo output = splitter(domain,12);
    REQUIRE( output.size() <= 12);

    trace.info() << "Original domain: "<<domain<<std::endl;
    for(auto d: output)
        trace.info()<< "   subdomain: "<<d.domain<<std::endl;

#ifdef DGTAL_TESTS_WITH_VIEWER
#ifdef DGTAL_WITH_POLYSCOPE_VIEWER
    PolyscopeViewer viewer;
    HueShadeColorMap<unsigned int> cmap(0,(unsigned int)output.size());
    for(auto i=0; i< output.size(); ++i)
    {
        viewer << cmap(i);
        viewer << output[i].domain;
    }
    viewer.show();
#endif
#endif
}

TEST_CASE( "Domain Axis Splitter tests" )
{
    Domain domain(Point(0,0,0), Point(16,32,64));

    AxisDomainSplitter<Domain> splitter;

    AxisDomainSplitter<Domain>::SplitDomainsInfo output = splitter(domain,3,0);
    REQUIRE( output.size() == 3);

    trace.info() << "Original domain: "<<domain<<std::endl;
    for(auto d: output)
        trace.info()<< "   subdomain: "<<d.domain<<std::endl;

#ifdef DGTAL_TESTS_WITH_VIEWER
#ifdef DGTAL_WITH_POLYSCOPE_VIEWER
    PolyscopeViewer viewer;
    HueShadeColorMap<unsigned int> cmap(0,(unsigned int)output.size());
    for(auto i=0; i< output.size(); ++i)
    {
        viewer << cmap(i);
        viewer << output[i].domain;
    }
    viewer.show();
#endif
#endif
}

TEST_CASE( "Domain Axis Splitter tests (another direction)" )
{
  Domain domain(Point(10,10,10), Point(16,32,64));

  AxisDomainSplitter<Domain> splitter;

  AxisDomainSplitter<Domain>::SplitDomainsInfo output = splitter(domain,2,1);
  REQUIRE( output.size() == 2);

  trace.info() << "Original domain: "<<domain<<std::endl;
  for(auto d: output)
    trace.info()<< "   subdomain: "<<d.domain<<std::endl;
}
