#if !defined(__DEC_DUALITY_LINEAR_ALGEBRA__)
#define __DEC_DUALITY_LINEAR_ALGEBRA__

#include "CContainer.h"

#include <boost/concept_check.hpp>
#include <DGtal/base/Common.h>
#include <DGtal/base/ConceptUtils.h>

template <typename T>
struct CDualityLinearAlgebra //: boost::CopyConstructible<T>
{
    public:
        typedef typename T::Container Container;
        typedef typename T::Calculus Calculus;
        typedef typename Calculus::Scalar Scalar;

        BOOST_CONCEPT_ASSERT(( DGtal::CContainer<Container> ));

        BOOST_CONCEPT_USAGE(CDualityLinearAlgebra)
        {
            T t0(calculus_const_ref);
            T t1(calculus_const_ref, container_const_ref);
            T t3 = t0 + t1;
            T t4 = scalar * t0;
        }
    private:
        const Container& container_const_ref;
        const Calculus& calculus_const_ref;
        Scalar scalar;

};

#endif

