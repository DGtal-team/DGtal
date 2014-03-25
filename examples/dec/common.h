#if !defined(__DEC_EXAMPLE_COMMON_H__)
#define __DEC_EXAMPLE_COMMON_H__

#include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
using namespace DGtal;

inline DiscreteExteriorCalculus<Z2i::Domain, EigenSparseLinearAlgebraBackend> generateRing(const Z2i::Domain& domain)
{
    Z2i::DigitalSet set(domain);

    // create initial set
    for (int ii=0; ii<3; ii++)
        for (int jj=2; jj<8; jj++)
        {
            set.insert(Z2i::Point(jj,ii+1));
            set.insert(Z2i::Point(jj,ii+6));
            set.insert(Z2i::Point(ii+1,jj));
            set.insert(Z2i::Point(ii+6,jj));
        }

    //for (int ii=0; ii<10; ii++)
    //    for (int jj=0; jj<10; jj++)
    //        set.insert(Z2i::Point(ii,jj));

    //set.insert(Z2i::Point(6,2));
    //set.insert(Z2i::Point(5,3));
    //set.insert(Z2i::Point(7,3));
    //set.insert(Z2i::Point(6,4));

    //set.insert(Z2i::Point(4,4));
    //set.insert(Z2i::Point(4,5));
    //set.insert(Z2i::Point(5,4));
    //set.insert(Z2i::Point(5,5));

    return DiscreteExteriorCalculus<Z2i::Domain, EigenSparseLinearAlgebraBackend>(set);
}

#include <Eigen/SparseCholesky>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>

std::ostream& operator<<(std::ostream& os, const Eigen::ComputationInfo& info)
{
    os << "solve_info = ";

    switch (info)
    {
        case Eigen::Success:
            os << "success";
            break;
        case Eigen::NumericalIssue:
            os << "numerical_issue";
            break;
        case Eigen::NoConvergence:
            os << "no_convergence";
            break;
        case Eigen::InvalidInput:
            os << "invalid_input";
            break;
    }

    return os;
}

#endif

