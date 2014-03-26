/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#pragma once

/**
 * @file KForm.h
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2014/03/26
 *
 * Header file for module KForm.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(KForm_RECURSES)
#error Recursive header files inclusion detected in KForm.h
#else // defined(KForm_RECURSES)
/** Prevents recursive inclusion of headers. */
#define KForm_RECURSES

#if !defined KForm_h
/** Prevents repeated inclusion of headers. */
#define KForm_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/dec/Duality.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class KForm
  /**
   * Description of template class 'KForm' <p>
   * \brief Aim:
   * KForm represents kforms in the dec package.
   *
   * @tparam Calculus should be DiscreteExteriorCalculus.
   * @tparam order is the order of the kform.
   * @tparam duality is the duality of the kform.
   */
  template <typename C, Order order, Duality duality>
  class KForm
  {
    // ----------------------- Standard services ------------------------------
  public:
    typedef C Calculus;

    BOOST_STATIC_ASSERT(( order >= 0 ));
    BOOST_STATIC_ASSERT(( order <= Calculus::dimension ));

    typedef typename Calculus::Vector Container;
    typedef typename Calculus::Scalar Scalar;

    /**
     * Constructor.
     * @param calculus the discrete exterior calculus to use.
     */
    KForm(const Calculus& calculus);

    /**
     * Constructor.
     * @param calculus the discrete exterior calculus to use.
     * @param container the container to copy.
     */
    KForm(const Calculus& calculus, const Container& container);

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    KForm& operator=(const KForm& other);

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Container used to actually hold the kform data.
     */
    Container container;

    /**
     * Const reference to calculus
     */
    const Calculus& calculus;

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay(std::ostream& out) const;

    /**
     * Writes kform values to scell accumulator.
     * @param scell_map scell accumulator where the object is written.
     */
    template <typename Accum>
    void applyToAccum(Accum& scell_map) const;

    /**
     * Clear current kform.
     */
    void clear();

    /**
     * Get kcell from index.
     * @param index
     */
    typename Calculus::SCell getSCell(const typename Calculus::Index& index) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    KForm();

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class KForm

  /**
   * Overloads 'operator<<' for displaying objects of class 'KForm'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'KForm' to write.
   * @return the output stream after the writing.
   */
  template <typename Calculus, Order order, Duality duality>
  std::ostream&
  operator<<(std::ostream& out, const KForm<Calculus, order, duality>& object);

  /**
   * Overloads 'operator+' for adding objects of class 'KForm'.
   * @return form_a + form_b.
   */
  template <typename Calculus, Order order, Duality duality>
  KForm<Calculus, order, duality>
  operator+(const KForm<Calculus, order, duality>& form_a, const KForm<Calculus, order, duality>& form_b);

  /**
   * Overloads 'operator-' for substracting objects of class 'KForm'.
   * @return form_a - form_b.
   */
  template <typename Calculus, Order order, Duality duality>
  KForm<Calculus, order, duality>
  operator-(const KForm<Calculus, order, duality>& form_a, const KForm<Calculus, order, duality>& form_b);

  /**
   * Overloads 'operator*' for scalar multiplication of objects of class 'KForm'.
   * @return scalar * form.
   */
  template <typename Calculus, Order order, Duality duality>
  KForm<Calculus, order, duality>
  operator*(const typename Calculus::Scalar& scalar, const KForm<Calculus, order, duality>& form);

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/dec/KForm.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined KForm_h

#undef KForm_RECURSES
#endif // else defined(KForm_RECURSES)
