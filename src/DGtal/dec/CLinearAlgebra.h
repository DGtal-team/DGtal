#if !defined(__DEC_LINEAR_ALGEBRA_CONCEPT__)
#define __DEC_LINEAR_ALGEBRA_CONCEPT__

#include "CContainer.h"

#include <boost/concept_check.hpp>
#include <DGtal/base/Common.h>
#include <DGtal/base/ConceptUtils.h>

template <typename T>
struct CLinearAlgebra
{
    public:
        typedef typename T::Index Index;
        typedef typename T::Scalar Scalar;
        typedef typename T::Vector Vector;
        typedef typename T::Matrix Matrix;

        BOOST_CONCEPT_ASSERT(( CContainer<Vector> ));
        BOOST_CONCEPT_ASSERT(( CContainer<Matrix> ));

        BOOST_CONCEPT_USAGE(CLinearAlgebra)
        {
        }
    private:

};

#endif

