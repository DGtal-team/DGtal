#if !defined(__DEC_EXAMPLE_COMMON_H__)
#define __DEC_EXAMPLE_COMMON_H__

#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
using namespace DGtal;
using namespace Z2i;

inline DigitalSet generateRingSet(const Domain& domain)
{
    DigitalSet set(domain);
    RealPoint center = domain.lowerBound() + domain.upperBound();
		center /= 2.;
    RealPoint delta = domain.upperBound() - domain.lowerBound();
    double radius = delta[0]>delta[1] ? delta[1] : delta[0];
    radius += 1.;
    radius /= 2.;

    for (Domain::ConstIterator di=domain.begin(), die=domain.end(); di!=die; di++)
    {
        const Point point = *di;
        const RealPoint point_real = RealPoint(point) - center;
        if (point_real.norm() < 1.*radius/6.) continue;
        if (point_real.norm() > 5.*radius/6.) continue;
        set.insert(point);
    }

    return set;
}

inline DigitalSet generateDoubleRingSet(const Domain& domain)
{
    DigitalSet set(domain);
    RealPoint center = domain.lowerBound() + domain.upperBound();
		center /= 2.;
    RealPoint delta = domain.upperBound() - domain.lowerBound();
    double radius = delta[0]>delta[1] ? delta[1] : delta[0];
    radius += 1.;
    radius /= 2.;

		center -= RealPoint(radius/2.,0);
    for (Domain::ConstIterator di=domain.begin(), die=domain.end(); di!=die; di++)
    {
        const Point point = *di;
        const RealPoint point_real = RealPoint(point) - center;
        if (point_real.norm() < 1.*radius/6.) continue;
        if (point_real.norm() > 5.*radius/6.) continue;
        set.insert(point);
    }

		center += RealPoint(radius,0);
    for (Domain::ConstIterator di=domain.begin(), die=domain.end(); di!=die; di++)
    {
        const Point point = *di;
        const RealPoint point_real = RealPoint(point) - center;
        if (point_real.norm() < 1.*radius/6.) continue;
        if (point_real.norm() > 5.*radius/6.) continue;
        set.insert(point);
    }

    return set;
}

#endif

