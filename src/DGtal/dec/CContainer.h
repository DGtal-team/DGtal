#if !defined(__DEC_CONTAINER_CONCEPT__)
#define __DEC_CONTAINER_CONCEPT__

#include <boost/concept_check.hpp>
#include <DGtal/base/Common.h>
#include <DGtal/base/ConceptUtils.h>
#include <DGtal/kernel/CEuclideanRing.h>

namespace DGtal
{

    template <typename T>
    struct CContainer : boost::Assignable<T>, boost::DefaultConstructible<T>
    {
        public:
            typedef typename T::Scalar Scalar;
            typedef typename T::Index Index;

            //BOOST_CONCEPT_ASSERT(( CEuclideanRing<Scalar> ));

            BOOST_CONCEPT_USAGE(CContainer)
            {
                z.clear();
                ConceptUtils::sameType(z, T(x + y));
                ConceptUtils::sameType(z, T(a * x));
                ConceptUtils::sameType(ii, x.rows());
                ConceptUtils::sameType(ii, x.cols());
            }

        private:
            const T x,y;
            T z;
            Scalar a;
            Index ii;
    };

    template <typename T>
    struct CMatrix : CContainer<T>
    {
        public:
            typedef typename T::Scalar Scalar;
            typedef typename T::Index Index;

            BOOST_CONCEPT_USAGE(CMatrix)
            {
                ConceptUtils::sameType(a, x(ii, jj));
                ConceptUtils::sameType(a_ref, z(ii, jj));
            }

        private:
            const T x;
            T z;
            Scalar a;
            Scalar& a_ref;
            Index ii, jj;
    };

    template <typename T>
    struct CVector : CContainer<T>
    {
        public:
            typedef typename T::Scalar Scalar;
            typedef typename T::Index Index;

            BOOST_CONCEPT_USAGE(CVector)
            {
                ConceptUtils::sameType(a, x(ii));
                ConceptUtils::sameType(a_ref, z(ii));
            }

        private:
            const T x;
            T z;
            Scalar a;
            Scalar& a_ref;
            Index ii;
    };
}

#endif

