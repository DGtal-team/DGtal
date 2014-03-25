#if !defined(__DEC_EXAMPLE_COMMON_H__)
#define __DEC_EXAMPLE_COMMON_H__

#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/math/linalg/CLinearAlgebraSolver.h"
#include "DGtal/dec/DECSolver.h"

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
using namespace DGtal;

inline DiscreteExteriorCalculus<Z2i::Domain, EigenSparseLinearAlgebraBackend> generateRing(const Z2i::Domain& domain)
{
    Z2i::DigitalSet set(domain);

    // create ring
    for (int ii=0; ii<3; ii++)
        for (int jj=2; jj<8; jj++)
        {
            set.insert(Z2i::Point(jj,ii+1));
            set.insert(Z2i::Point(jj,ii+6));
            set.insert(Z2i::Point(ii+1,jj));
            set.insert(Z2i::Point(ii+6,jj));
        }

    //// fill domain
    //for (int ii=0; ii<10; ii++)
    //    for (int jj=0; jj<10; jj++)
    //        set.insert(Z2i::Point(ii,jj));

    return DiscreteExteriorCalculus<Z2i::Domain, EigenSparseLinearAlgebraBackend>(set);
}

#endif

