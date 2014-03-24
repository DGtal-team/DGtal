
inline void
clear()
{
}

inline Scalar&
operator()(Index ii, Index jj)
{
    return coeffRef(ii, jj);
}

inline Scalar
operator()(Index ii, Index jj)
const
{
    return coeff(ii, jj);
}

