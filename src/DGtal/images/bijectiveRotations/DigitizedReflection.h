

#ifndef DIGITIZEDREFLECTION_H
#define DIGITIZEDREFLECTION_H
#include "GAVector.h"

namespace DGtal {
    template<typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point,
    typename TFunctor = DGtal::functors::VectorRounding < TInputValue, TOutputValue >>
    struct Reflection
    {
        GAVector<TSpace> normalVector;
        TFunctor functor;

        explicit Reflection ( const GAVector<TSpace> & m)
            :normalVector(m){}

        /**
       * Operator
       * @return the reflected and digitized point.
       */
        inline
        TOutputValue operator()( const TInputValue & aInput ) const
        {
            Z2i::RealPoint m_r = Z2i::RealPoint(normalVector.my_gavec[0],normalVector.my_gavec[1]);
            Z2i::RealPoint x_r = Z2i::RealPoint(aInput[0],aInput[1]); // \todo change type of xr
            Z2i::RealPoint p=x_r - 2.0*((x_r[0]*m_r[0] + x_r[1]*m_r[1])/(m_r[0]*m_r[0] + m_r[1]*m_r[1]))*m_r;
            return functor ( p );
        }

    };
}
#endif //DIGITIZEDREFLECTION_H
