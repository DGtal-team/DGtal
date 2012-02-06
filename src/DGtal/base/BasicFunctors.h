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
 * @file BasicFunctors.h
 * @author Guillaume Damiand (\c guillaume.damiand@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2010/07/02
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/01/27
 *
 *
 * Header file for module BasicFunctors.h
 *
 * This file contains the definition of basic functors.
 *
 * This file is part of the DGtal library.
 */

#if defined(BasicFunctors_RECURSES)
#error Recursive header files inclusion detected in BasicFunctors.h
#else // defined(BasicFunctors_RECURSES)
/** Prevents recursive inclusion of headers. */
#define BasicFunctors_RECURSES

#if !defined BasicFunctors_h
/** Prevents repeated inclusion of headers. */
#define BasicFunctors_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <algorithm>
#include <functional>
#include <cmath>

#include "boost/concept_check.hpp"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{

/////////////////////////////////////////////////////////////////////////////
/// Duplicated STL functors
/////////////////////////////////////////////////////////////////////////////
  template<typename T>
  struct MinFunctor
  {
    inline
    T operator() (const T&a, const T&b) const
    { return std::min(a,b); }
  };
  
  template<typename T>
  struct MaxFunctor
  {
    inline
    T operator() (const T&a, const T&b) const
    { return std::max(a,b); }
  };

 /**
   * Copy of the std::minus binary operator (not implemented on MS-VS)
   */
  template <class T> 
  struct MinusFunctor : binary_function <T,T,T>
  {
    T operator() (const T& x, const T& y) const
    {return x-y;}
  };

  /**
   * Abs functor. 
   */
  template <class T>
  struct AbsFunctor : unary_function<T,T>
  { 
   inline
   T operator() (const T &x) const
    {
      if (x < 0)
  return -x;
      else 
  return x;
    }
  };
}
///////////////////////////////////////////////////////////////////////////////
// Some basic unary functors that may be useful
//////////////////////////////////////////////////////////////////////////////
  /**
   * Description of template class 'DefaultFunctor' <p>
   * \brief Aim: Define a simple default functor that 
   * just returns its argument 
   *
   */
  struct DefaultFunctor
  {
    /** 
     * Operator
     * @return @a aT.
     * @tparam T any type
     */
    template <typename T >
    inline
    T operator()(const T& aT) const
    {
      return aT;
    }
  };

  /**
   * Description of template class 'ConstValueFunctor' <p>
   * \brief Aim: Define a simple functor that returns 
   * a constant value (0 by default). 
   *
   * @tparam TValue type of the value
   */
  template <typename TValue>
  class ConstValueFunctor
  {
  public:
    typedef TValue Value;

    /** 
     * Constructor.
     * @param value  the constant value.
     */
    ConstValueFunctor(const Value& aValue = 0)
      :myValue(aValue) {};
    
    /** 
     * Operator
     *
     * @tparam TInput type of the input object
     * @param aInput input object
     *
     * @return the constant value.
     */
    template <typename TInput>
    inline
    Value operator()(const TInput& aInput) const
    {
      return myValue;
    }

  private:
    /** 
     * value
     */
    Value myValue;
    
  };


  /**
   * Description of template class 'CastFunctor' <p>
   * \brief Aim: Define a simple functor using the static cast operator. 
   *
   * @tparam TOutput type of the return value
   */
  template <typename TOutput >
  struct CastFunctor
  {
    /** 
     * Operator
     * @return the conversion of @a aInput into an object of type TOutput.
     * @tparam TInput type of the input value
     */
    template<typename TInput>
    inline
    TOutput operator()(const TInput& aInput) const
    {
      return static_cast<TOutput>(aInput);
    }
  }; 


  /**
   * Description of template class 'Composer' <p>
   * \brief Aim: Define a new Functor from the composition of two other functors
   *
   * @tparam TFunctor1 first functor
   * @tparam TFunctor2 second functor
   * @tparam ReturnType
   */
  template <typename TFunctor1, typename TFunctor2, typename ReturnType >
  class Composer
  {
  public:
    /** 
     * Constructor
     * @param aF1 any Functor
     * @param aF2 any Functor
     */
    Composer(TFunctor1& aF1, TFunctor2& aF2);
    /** 
     * Copy Operator
     * @param other object to copy
     */
    Composer(const Composer& other);
    /** 
     * Assignement Operator
     * @param other object to copy
     */
    Composer& operator=(const Composer& other);

    /** 
     * Operator ()
     *
     * NB: @a myF2 return type should be equal to 
     * (or implicitly castable into) ReturnType
     *
     * @return object of type ReturnType coming from 
     * the composition @a myF1 o @a myF2 on @a aInput, 
     * ie. myF2 ( myF1 ( aInput ) )
     *
     * @tparam TInput type of the input value
     */
    template<typename TInput>
    inline
    ReturnType operator()(const TInput& aInput) const
    {
      return myF2( myF1 ( aInput ) );
    }

  private:
    /** 
     * First Functor.
     */
    TFunctor1& myF1;
    /** 
     * Second Functor.
     */
    TFunctor2& myF2;
  };

template<typename TFunctor1, typename TFunctor2, typename ReturnType>
inline
Composer<TFunctor1, TFunctor2, ReturnType>
::Composer(TFunctor1& aF1, TFunctor2& aF2)
 :myF1(aF1), myF2(aF2) 
{};

template<typename TFunctor1, typename TFunctor2, typename ReturnType>
inline
Composer<TFunctor1, TFunctor2, ReturnType>
::Composer(const Composer<TFunctor1, TFunctor2, ReturnType>& other)
 :myF1(other.myF1), myF2(other.myF2)
{};


template<typename TFunctor1, typename TFunctor2, typename ReturnType>
inline
Composer<TFunctor1, TFunctor2, ReturnType>&
Composer<TFunctor1, TFunctor2, ReturnType>
::operator=(const Composer<TFunctor1, TFunctor2, ReturnType>& other)
{
  myF1 = other.myF1; 
  myF2 = other.myF2; 
};

  /**
   * Description of template class 'BinaryToUnaryFunctor' <p>
   * \brief Aim: Define a simple functor that returns 
   * the result of an operation between a given value
   * and a threshold value (0 by default). 
   *
   * @tparam TValue a type that should support
   * the operation performed by TBinaryFunctor. 
   * @tparam TBinaryFunctor a model of binary function
   * (default MinusFunctor)
   */
  template <typename TValue, typename TBinaryFunctor = MinusFunctor<TValue> >
  class BinaryToUnaryFunctor
  {
  public:
    typedef TValue Value;

    /** 
     * Constructor.
     * @param value  the threshold value.
     * @param aF  the binary functor.
     */
    BinaryToUnaryFunctor(const Value& aValue = 0, const TBinaryFunctor& aF = TBinaryFunctor() )
      :myValue(aValue), myF(aF) {};
    
    /** 
     * Operator
     * @return binary function return value
     * (with arguments @a aValue  and  @a myValue ).
     */
    Value operator()(const Value& aValue) const
    {
      return myF(aValue, myValue);
    }

  private:
    /** 
     * Threshold value
     */
    Value myValue;
    /** 
     * Binary functor
     */
    TBinaryFunctor myF;
    
  };

/**
 * // template class Thresholder
 * \brief Aim: A small functor with an operator ()
 * that compares one value to a threshold value 
 * according to two bool template parameters.
 *
 * @tparam T  type for a value that must be equally and less-than comparable
 * @tparam isLower  a bool for the comparison sign : 
 * 'true' for < (default), 'false' for <
 * @tparam isEqual  a bool for the equality :
 * 'true' for a large inequality (default), 'false' for a strict one
 *
 */
template <typename T, bool isLower = true, bool isEqual = true >
class Thresholder {
  public:
    BOOST_CONCEPT_ASSERT(( boost::EqualityComparable<T> ));
    BOOST_CONCEPT_ASSERT(( boost::LessThanComparable<T> ));

    typedef T Input; 

    /** 
     * Constructor. 
     * @param value  the threshold value (default 0).
     */
    Thresholder(const Input& aT = 0):myT(aT) {};
    /**
    * Compares  @a aI to @ myT.
    * @param aI  any input value
    * @return 'true' or 'false' according to isLower and isEqual
    */
    bool operator()(const Input& aI) const {
      std::less_equal<Input> c;
      return c(aI,myT); 
    }
  private:
    /** 
     * Threshold value
     */
   Input myT;
};

//specializations
template <typename T>
struct Thresholder<T,false,false> {

  public:
    BOOST_CONCEPT_ASSERT(( boost::EqualityComparable<T> ));
    BOOST_CONCEPT_ASSERT(( boost::LessThanComparable<T> ));

    typedef T Input; 

    Thresholder(const Input& aT = 0):myT(aT) {};

    bool operator()(const Input& aI) const {
    std::greater<Input> c;
    return c(aI,myT);
    }

  private:
    Input myT;
};
template <typename T>
struct Thresholder<T,false,true> {
  public:
    BOOST_CONCEPT_ASSERT(( boost::EqualityComparable<T> ));
    BOOST_CONCEPT_ASSERT(( boost::LessThanComparable<T> ));

    typedef T Input; 

    Thresholder(const Input& aT = 0):myT(aT) {};
    bool operator()(const Input& aI) const {
    std::greater_equal<Input> c;
    return c(aI,myT);
    }

  private:
    Input myT;
};

template <typename T>
struct Thresholder<T,true,false> {
  public:
    BOOST_CONCEPT_ASSERT(( boost::EqualityComparable<T> ));
    BOOST_CONCEPT_ASSERT(( boost::LessThanComparable<T> ));

    typedef T Input; 

    Thresholder(const Input& aT = 0):myT(aT) {};

    bool operator()(const Input& aI) const {
    std::less<Input> c;
    return c(aI,myT);
    }

  private:
    Input myT;
};

template <typename T>
struct Thresholder<T,true,true> {
  public:
    BOOST_CONCEPT_ASSERT(( boost::EqualityComparable<T> ));
    BOOST_CONCEPT_ASSERT(( boost::LessThanComparable<T> ));

    typedef T Input; 

    Thresholder(const Input& aT = 0):myT(aT) {};

    bool operator()(const Input& aI) const {
    std::less_equal<Input> c;
    return c(aI,myT);
    }

  private:
    Input myT;
};


///////////////////////////////////////////////////////////////////////////////


#endif // !defined BasicFunctors_h

#undef BasicFunctors_RECURSES
#endif // else defined(BasicFunctors_RECURSES)
