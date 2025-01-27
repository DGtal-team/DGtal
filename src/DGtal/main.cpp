#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetSelector.h"
#include "DGtal/topology/DigitalTopology.h"
#include "DGtal/topology/MetricAdjacency.h"
#include "DGtal/topology/Object.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/KhalimskyPreSpaceND.h"
#include "DGtal/geometry/curves/GridCurve.h"
#include "DGtal/geometry/volumes/distance/ExactPredicateLpSeparableMetric.h"
#include "DGtal/geometry/volumes/distance/ExactPredicateLpPowerSeparableMetric.h"

template <DGtal::concepts::CConstSinglePassRange T>
int test()
{
	std::cout << "Hello World !" << std::endl;
	return 0;
}

int test2()
{	
	test<typename DGtal::KhalimskySpaceND<2>::Cells>();
	return 0;
}