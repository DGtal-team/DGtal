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
 * @file CellularConvolutionKernelFromSet.h
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/04/05
 *
 * Header file for module CellularConvolutionKernelFromSet.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CellularConvolutionKernelFromSet_RECURSES)
#error Recursive header files inclusion detected in CellularConvolutionKernelFromSet.h
#else // defined(CellularConvolutionKernelFromSet_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CellularConvolutionKernelFromSet_RECURSES

#if !defined CellularConvolutionKernelFromSet_h
/** Prevents repeated inclusion of headers. */
#define CellularConvolutionKernelFromSet_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/CCellularConvolutionKernel.h"
#include "DGtal/kernel/sets/CDigitalSet.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CellularConvolutionKernelFromSet
  /**
   * Description of class 'CellularConvolutionKernelFromSet' <p>
   * \brief Aim:
   */
  template<typename TDigitalSet, typename TKSpace, typename TCellFunctor>
  class CellularConvolutionKernelFromSet
  {

      // ----------------------- Standard services ------------------------------

    public:
      BOOST_CONCEPT_ASSERT(( CDigitalSet<TDigitalSet> ) );

      typedef TDigitalSet DigitalSet;
      typedef TKSpace KSpace;
      typedef typename KSpace::SCell Spel;
      typedef typename KSpace::SCells Spels;
      typedef typename Spels::ConstIterator ConstIterator;
      typedef typename TCellFunctor::Quantity Quantity;
      typedef TCellFunctor CellFunctor;
      
      ///@todo BOOST ASSERT CEllFunctor::KSpace == KSpace
      
      CellularConvolutionKernelFromSet( const DigitalSet &aDigitalSet, const Spel &aOrigin, const CellFunctor&aFunctor ): myOrigin( aOrigin ), myCellFunctor(aFunctor)
      {
        KSpace myKSpace;
        bool space_ok = myKSpace.init( aDigitalSet.domain().lowerBound(), aDigitalSet.domain().upperBound(), true );
        ASSERT( space_ok );

        //Explicit copy of set points into Spel

        for ( typename DigitalSet::ConstIterator it = aDigitalSet.begin(), itend = aDigitalSet.end();
              it != itend;
              ++it )
          mySpels.push_back( myKSpace.sSpel( *it , true ) );
      }

      /**
       * Destructor.
       */
      ~CellularConvolutionKernelFromSet();

      // ----------------------- Interface --------------------------------------

    public:


      const Spel & origin()  const
      {
        return myOrigin;
      }


      ConstIterator begin() const
      {
        return mySpels.begin();
      }


      ConstIterator end() const
      {
        return mySpels.end();
      }


      Quantity operator()( const Spel &aSpel ) const
      {
        return myCellFunctor(aSpel);
      }


      /**
       * Writes/Displays the object on an output stream.
       * @param out the output stream where the object is written.
       */
      void selfDisplay( std::ostream & out ) const;

      /**
       * Checks the validity/consistency of the object.
       * @return 'true' if the object is valid, 'false' otherwise.
       */
      bool isValid() const;

      // ------------------------- Protected Datas ------------------------------

    private:
      // ------------------------- Private Datas --------------------------------

    private:

      ///
      Spels mySpels;

      const Spel & myOrigin;

      const CellFunctor &myCellFunctor;
      
      // ------------------------- Hidden services ------------------------------

    protected:

      /**
       * Constructor.
       * Forbidden by default (protected to avoid g++ warnings).
       */
      CellularConvolutionKernelFromSet();

    private:

      /**
       * Copy constructor.
       * @param other the object to clone.
       * Forbidden by default.
       */
      CellularConvolutionKernelFromSet( const CellularConvolutionKernelFromSet & other );

      /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       * Forbidden by default.
       */
      CellularConvolutionKernelFromSet & operator= ( const CellularConvolutionKernelFromSet & other );

      // ------------------------- Internals ------------------------------------

    private:

  }; // end of class CellularConvolutionKernelFromSet


  /**
   * Overloads 'operator<<' for displaying objects of class 'CellularConvolutionKernelFromSet'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'CellularConvolutionKernelFromSet' to write.
   * @return the output stream after the writing.
   */
  template <typename Set,typename Space,typename Functor>
  std::ostream&
  operator<< ( std::ostream & out, const CellularConvolutionKernelFromSet<Set,Space,Functor> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/CellularConvolutionKernelFromSet.ih"



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CellularConvolutionKernelFromSet_h

#undef CellularConvolutionKernelFromSet_RECURSES
#endif // else defined(CellularConvolutionKernelFromSet_RECURSES)
