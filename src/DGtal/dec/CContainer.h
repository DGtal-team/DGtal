#if !defined(__DEC_CONTAINER_CONCEPT__)
#define __DEC_CONTAINER_CONCEPT__

#include "Duality.h"

#include <boost/concept_check.hpp>
#include <DGtal/base/Common.h>
#include <DGtal/base/ConceptUtils.h>

template <typename T>
struct CContainer : boost::Assignable<T>
{
		public:
				typedef typename T::Value Value;

				BOOST_CONCEPT_USAGE(CContainer)
				{
						z = x + y;
						z = a * x;
				}
		private:
				const T x,y;
				T z;
				Value a;
};


#endif

