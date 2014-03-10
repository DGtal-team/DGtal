#if !defined(__DEC_VECTOR_FIELD_H__)
#define __DEC_VECTOR_FIELD_H__

#include "Duality.h"
#include "KForm.h"

#include <boost/array.hpp>

template <typename Calculus, Duality duality>
struct VectorField
{
    typedef typename Calculus::Dimension Dimension;
    typedef typename Calculus::LinearAlgebraBackend::VectorFieldCoordinate Coordinate;

    typedef boost::array<Coordinate, Calculus::dimension> Coordinates;
    Coordinates coordinates;
    const Calculus& calculus;

    VectorField(const Calculus& _calculus);

    template <typename Board>
    void
    display2D(Board& board, const double& scale = .25) const;

    KForm<Calculus, 0, duality>
    extractZeroForm(const Dimension& dim) const;
};

template <typename Calculus, Duality duality>
std::ostream&
operator<<(std::ostream& os, const VectorField<Calculus, duality>& field);

#include "VectorField.ih"

#endif

