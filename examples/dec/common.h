#if !defined(__DEC_EXAMPLE_COMMON_H__)
#define __DEC_EXAMPLE_COMMON_H__

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
using namespace DGtal;

inline Z2i::DigitalSet generateRingSet(const Z2i::Domain& domain)
{
    Z2i::DigitalSet set(domain);
    Z2i::RealPoint center = domain.lowerBound() + domain.upperBound();
    center /= 2.;
    Z2i::RealPoint delta = domain.upperBound() - domain.lowerBound();
    double radius = delta[0]>delta[1] ? delta[1] : delta[0];
    radius += 1.;
    radius /= 2.;

    for (Z2i::Domain::ConstIterator di=domain.begin(), die=domain.end(); di!=die; di++)
    {
        const Z2i::Point point = *di;
        const Z2i::RealPoint point_real = Z2i::RealPoint(point) - center;
        if (point_real.norm() < 1.*radius/6.) continue;
        if (point_real.norm() > 5.*radius/6.) continue;
        set.insert(point);
    }

    return set;
}

inline Z2i::DigitalSet generateDoubleRingSet(const Z2i::Domain& domain)
{
    Z2i::DigitalSet set(domain);
    Z2i::RealPoint center = domain.lowerBound() + domain.upperBound();
    center /= 2.;
    Z2i::RealPoint delta = domain.upperBound() - domain.lowerBound();
    double radius = delta[0]>delta[1] ? delta[1] : delta[0];
    radius += 1.;
    radius /= 2.;

    center -= Z2i::RealPoint(radius/2.+radius/5.,0);
    for (Z2i::Domain::ConstIterator di=domain.begin(), die=domain.end(); di!=die; di++)
    {
        const Z2i::Point point = *di;
        const Z2i::RealPoint point_real = Z2i::RealPoint(point) - center;
        if (point_real.norm() < 1.*radius/6.) continue;
        if (point_real.norm() > 4.5*radius/6.) continue;
        set.insert(point);
    }

    center += Z2i::RealPoint(2*(radius/2.+radius/5.),0);
    for (Z2i::Domain::ConstIterator di=domain.begin(), die=domain.end(); di!=die; di++)
    {
        const Z2i::Point point = *di;
        const Z2i::RealPoint point_real = Z2i::RealPoint(point) - center;
        if (point_real.norm() < 1.*radius/6.) continue;
        if (point_real.norm() > 4.5*radius/6.) continue;
        set.insert(point);
    }

    return set;
}

#endif

