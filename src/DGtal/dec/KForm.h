#if !defined(__DEC_KFORM__)
#define __DEC_KFORM__

#include "Duality.h"

#include <DGtal/base/Common.h>

template <typename C, Order order, Duality duality>
struct KForm
{
    typedef C Calculus;
    typedef typename Calculus::LinearAlgebra::Vector Container;

    typedef typename Calculus::SCell SCell;
    typedef typename Calculus::Index Index;

    BOOST_STATIC_ASSERT(( order >= 0 ));
    BOOST_STATIC_ASSERT(( order <= Calculus::dimension ));

    const Calculus& calculus;
    Container container;

    KForm(const Calculus& _calculus);
    KForm(const Calculus& _calculus, const Container& _container);

    template <typename SCellMap>
    void applyToSCellMap(SCellMap& scell_map) const;

    SCell
    getSCell(const Index& index) const;
};

template <typename Calculus, Order order, Duality duality>
std::ostream&
operator<<(std::ostream& os, const KForm<Calculus, order, duality>& form);

// k-forms addition
template <typename Calculus, Order order, Duality duality>
KForm<Calculus, order, duality>
operator+(const KForm<Calculus, order, duality>& form_a, const KForm<Calculus, order, duality>& form_b);

// k-forms scalar multiplication
template <typename Calculus, Order order, Duality duality>
KForm<Calculus, order, duality>
operator*(const typename Calculus::Scalar& scalar, const KForm<Calculus, order, duality>& form);

#include "KForm.ih"

#endif

