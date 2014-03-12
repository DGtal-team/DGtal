#if !defined(__Calculus_OPERATOR_H__)
#define __Calculus_OPERATOR_H__

#include "KForm.h"

template <typename C, Order order_in, Duality duality_in, Order order_out, Duality duality_out>
struct LinearOperator
{
    typedef C Calculus;
    typedef typename Calculus::LinearAlgebra::Matrix Container;

    const Calculus& calculus;

    Container container;

    typedef KForm<Calculus, order_in, duality_in> InputKForm;
    typedef KForm<Calculus, order_out, duality_out> OutputKForm;

    LinearOperator(const Calculus& _calculus);
    LinearOperator(const Calculus& _calculus, const Container& _container);
};

// composition operator between linear operator
template <typename Calculus, Order order_in, Duality duality_in, Order order_fold, Duality duality_fold, Order order_out, Duality duality_out>
LinearOperator<Calculus, order_in, duality_in, order_out, duality_out>
operator*(const LinearOperator<Calculus, order_fold, duality_fold, order_out, duality_out>& operator_left, const LinearOperator<Calculus, order_in, duality_in, order_fold, duality_fold>& operator_right);

// linear operator scalar multiplication
template <typename Calculus, Order order_in, Duality duality_in, Order order_out, Duality duality_out>
LinearOperator<Calculus, order_in, duality_in, order_out, duality_out>
operator*(const typename Calculus::Scalar& scalar, const LinearOperator<Calculus, order_in, duality_in, order_out, duality_out>& linear_operator);

// application operator on k-form
template <typename Calculus, Order order_in, Duality duality_in, Order order_out, Duality duality_out>
KForm<Calculus, order_out, duality_out>
operator*(const LinearOperator<Calculus, order_in, duality_in, order_out, duality_out>& linear_operator, const KForm<Calculus, order_in, duality_in>& input_form);

// linear operator addition
template <typename Calculus, Order order_in, Duality duality_in, Order order_out, Duality duality_out>
LinearOperator<Calculus, order_in, duality_in, order_out, duality_out>
operator+(const LinearOperator<Calculus, order_in, duality_in, order_out, duality_out>& linear_operator_a, const LinearOperator<Calculus, order_in, duality_in, order_out, duality_out>& linear_operator_b);

template <typename Calculus, Order order_in, Duality duality_in, Order order_out, Duality duality_out>
std::ostream&
operator<<(std::ostream& os, const LinearOperator<Calculus, order_in, duality_in, order_out, duality_out>& linear_operator);

#include "LinearOperator.ih"

#endif

