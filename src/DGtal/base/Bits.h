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
 * @file Bits.h
 * @brief A collection of functions to help with bitwise manipulations.
 * @author Nicolas Silva (\c nicolas.silva@insa-lyon.fr )
 *
 * @date 2010/09/02
 * 
 */
#ifndef BITS_HPP
#define BITS_HPP

#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/base/ExpressionTemplates.h"

namespace DGtal
{
 
 

  struct Bits
  {
    /**
     * @brief Bits Structs grouping all the functions of this tiny
     * library for bitwise calculation.
     *
     * @todo Check that T is CInteger.
     */

     
    /**
     * Returns a string containing value's bits. Mainly designed for debugging purposes.
     * 
     * @param value The value that you need to dipslay as a bit string.
     * @param nbBits number of bits to be displayed. If equal to 0, the number of bits will
     * correspond to the size of the type T.
     */
    template <typename T>
    static std::string bitString(T value, unsigned nbBits = 0)
    {
      std::string bitStr;
      /*MinFunctor<unsigned int> min;*/

      // if the requested number of bit is 0, use the size of the data type instead
      if(nbBits == 0) nbBits = sizeof(T)*8;
      int i = (int)(std::min((DGtal::int64_t)sizeof(T)*8-1, (DGtal::int64_t)nbBits-1));

      for(; i>=0; i--)
	{
	  T mask = ((T)1) << i; // if you take these parenthesis out,
	  // a mountain of incredible runtime
	  // errors will jump on you.(I warned
	  // ya !)
	  if(value & mask)
	    bitStr += "1" ;
	  else
	    bitStr += "0" ;
	}
      return bitStr;
    }


    // ---------------------------------------------------------------------
    //  Other functions
    // ---------------------------------------------------------------------

    /**
     * Returns an value which bits are of the form 0..010..0 with the nthBit equal to 1.
     */ 
    template<typename T> 
    static inline T mask(unsigned nthBit)
    {
      return static_cast<T>(static_cast<T>(1) << nthBit); 
    }

    /**
     *  Returns the state of key's nthBit bit.
     */ 
    template <typename T>
    static inline bool getBit(T key, unsigned nthBit)
    {
      return ( key & mask<T>(nthBit) );
    }

  
    /** 
     * Returns a value such that only its bit corresponding to the
     * first (least important) set bit of val, is set.
     */ 
    template <typename T>
    static  T firstSetBit(T val)
    {
      return (val & -val | val & (~val + 1));
    }


    /**
     * Returns a value such that only its bit corresponding to the
     * first (least important) unset bit of val, is set.
     */ 
    template <typename T>
    static T firstUnsetBit(T val)
    {
      return ~val & (val + 1);
    }


    /**
     * Returns the amount of set bits in val.
     */ 
    template <typename T>
    static unsigned nbSetBits(T val)
    {
      unsigned i = 0;
      for ( ; val; ++i) {val ^= val & -val; }
      return i;
    }


  };//struct
}
#endif
