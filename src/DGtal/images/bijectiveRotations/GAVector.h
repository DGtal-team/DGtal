//
// Created by stephane on 6/29/24.
//

#ifndef GAVECTOR_H
#define GAVECTOR_H
#include "DGtal/base/Common.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/kernel/domains/CDomain.h>

namespace DGtal {
    template<typename TSpace,typename TInputValue = typename TSpace::Point>
    struct GAVector{
        TInputValue my_gavec;

        explicit GAVector( const TInputValue& pt )
          : my_gavec( pt ) {}

        typename TInputValue::Component dot( GAVector other ) const
        {
            return my_gavec[0] * other.my_gavec[0] + my_gavec[1] * other.my_gavec[1];
        }

        typename TInputValue::Component bivectorPart( const GAVector& other ) const
        {
            return my_gavec[0] * other.my_gavec[1] - my_gavec[1] * other.my_gavec[0];
        }
        GAVector operator*( typename TInputValue::Component f ) const
        {
            return GAVector( TInputValue(my_gavec[0] * f, my_gavec[1] * f) );
        }

        GAVector operator*(const GAVector& v )// geometric product
        {
            return GAVector( TInputValue(this->dot(v), this->bivectorPart(v)) );
        }
        bool operator<( const GAVector& other ) const
        {
            return bivectorPart( other ) > 0;
        }

        double angleToXAxis() const
        {
            return atan2( my_gavec[1], my_gavec[0] );
        }

        bool operator==( const GAVector& other ) const
        {
            return bivectorPart( other ) == 0;
        }


    };
}


#endif //GAVECTOR_H
