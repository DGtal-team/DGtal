#if !defined(__DEC_LINEAR_ALGEBRA_CONCEPT__)
#define __DEC_LINEAR_ALGEBRA_CONCEPT__

#include "CContainer.h"

#include <boost/concept_check.hpp>
#include <DGtal/base/Common.h>
#include <DGtal/base/ConceptUtils.h>

namespace DGtal
{
    template <typename T>
    struct CLinearAlgebra
    {
        public:
            typedef typename T::Index Index;
            typedef typename T::Scalar Scalar;
            typedef typename T::Vector Vector;
            typedef typename T::Matrix Matrix;
            typedef typename T::Solver Solver;

            BOOST_CONCEPT_ASSERT(( CVector<Vector> ));
            BOOST_CONCEPT_ASSERT(( CMatrix<Matrix> ));
            BOOST_CONCEPT_ASSERT(( boost::DefaultConstructible<Solver> ));

            BOOST_CONCEPT_USAGE(CLinearAlgebra)
            {
                Vector foo = solver.compute(m).solve(v);
            }
        private:
            Solver solver;
            Vector v;
            Matrix m;
    };

}

#endif

