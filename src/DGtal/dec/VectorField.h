#if !defined(__DEC_VECTOR_FIELD_H__)
#define __DEC_VECTOR_FIELD_H__

#include "Duality.h"
#include "KForm.h"

#include <boost/array.hpp>

template <typename C, Duality duality>
struct VectorField
{
    typedef C Calculus;

    typedef typename Calculus::Dimension Dimension;
    typedef typename Calculus::Scalar Scalar;

    typedef boost::array<typename Calculus::LinearAlgebra::Vector, Calculus::dimension> Coordinates;

    Coordinates coordinates;
    const Calculus& calculus;

    VectorField(const Calculus& _calculus);
    VectorField(const Calculus& _calculus, const Coordinates& _coordinates);
    VectorField& operator=(const VectorField& _vector_field);

    template <typename Board>
    void
    display2D(Board& board, const double& scale = .25) const;

    KForm<Calculus, 0, duality>
    extractZeroForm(const Dimension& dim) const;

    void
    clear();
};

template <typename Calculus, Duality duality>
std::ostream&
operator<<(std::ostream& os, const VectorField<Calculus, duality>& field);

// vector fields addition
template <typename Calculus, Duality duality>
VectorField<Calculus, duality>
operator+(const VectorField<Calculus, duality>& _vector_field_a, const VectorField<Calculus, duality>& _vector_field_b);

// vector fields subtraction
template <typename Calculus, Duality duality>
VectorField<Calculus, duality>
operator-(const VectorField<Calculus, duality>& _vector_field_a, const VectorField<Calculus, duality>& _vector_field_b);

// vector fields scalar multiplication
template <typename Calculus, Duality duality>
VectorField<Calculus, duality>
operator*(const typename Calculus::Scalar& _scalar, const VectorField<Calculus, duality>& _vector_field);

#include "VectorField.ih"

#endif

