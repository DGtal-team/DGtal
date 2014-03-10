#if !defined(__DEC_COMMON__)
#define __DEC_COMMON__

#include <iostream>

enum Duality
{
    PRIMAL = 0,
    DUAL = 1,
};

inline std::ostream&
operator<<(std::ostream& os, const Duality& duality)
{
    switch (duality)
    {
        case PRIMAL:
            os << "primal";
            break;
        case DUAL:
            os << "dual";
            break;
    }
    return os;
}

typedef unsigned int Order;

#endif

