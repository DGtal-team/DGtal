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
 * @file AffineTransformation2D.h
 * @author Phuc Ngo (\c hoai-diem-phuc.ngo@loria.fr )
 * Laboratoire Lorrain de Recherche en Informatique et ses Applications (LORIA), France
 *
 * @date 01/05/2021
 *
 * This file is part of the DGtal library.
 */

#if defined(AffineTransformation2D_RECURSES)
#error Recursive header files inclusion detected in AffineTransformation2D.h
#else // defined(AffineTransformation2D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AffineTransformation2D_RECURSES

#if !defined AffineTransformation2D_h
/** Prevents repeated inclusion of headers. */
#define AffineTransformation2D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <cmath>
#include <climits>
#include <utility>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/kernel/domains/CDomain.h>
#include <DGtal/kernel/CSpace.h>
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/GeometricTransformation2D.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
namespace functors
{
/////////////////////////////////////////////////////////////////////////////
// Template class ForwardAffineTransformation2D
/**
     * Description of template functor like class 'ForwardAffineTransformation2D' <p>
     * \brief Aim: implements forward rigid transformation of point in the 2D integer space.
     * Warring: This version uses closest neighbor interpolation.
     *
     * @tparam TSpace a 2 dimensional space.
     * @tparam TInputValue type of the input point e.g., TSpace::RealPoint
     * @tparam TOutputValue type of the output point e.g., TSpace::Point
     * @tparam TFunctor a functor operating on the output e.g., a rounding function.
     *
     * @see exampleAffineTransformation3d.cpp
     */
template < typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point,
           typename TFunctor = VectorRounding < TInputValue, TOutputValue > >
class ForwardAffineTransformation2D : public GeometricTransformation2D <TSpace,TInputValue,TOutputValue,TFunctor>
{
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( concepts::CSpace<TSpace> ));
    BOOST_STATIC_ASSERT(( TSpace::dimension == 2 ));
    BOOST_STATIC_ASSERT(( TOutputValue::dimension == 2 ));
    BOOST_STATIC_ASSERT(( TInputValue::dimension == 2 ));

    // ----------------------- Types ------------------------------
public:
    typedef typename TSpace::RealPoint RealPoint;
    typedef typename TSpace::RealVector RealVector;
    typedef Eigen::Matrix<double, TSpace::dimension, TSpace::dimension> RealMatrix;
    
    // ----------------------- Interface --------------------------------------
public:
    /**
       * Constructor.
       * @param aOrigin  the center of affine transform.
       * @param aMatrix  the affine matrix.
       * @param aTranslate  the 2D dimensional vector which represents translation.
       */
    ForwardAffineTransformation2D ( const RealPoint & aOrigin, const RealMatrix & aMatrix, const RealVector & aTranslate )
    {
        this->origin = aOrigin;
        BOOST_ASSERT((aMatrix(0,0)*aMatrix(1,1)!=aMatrix(1,0)*aMatrix(0,1)));
        this->transform_matrix = aMatrix;
        this->translation = aTranslate;
    }

    /**
       * Constructor.
       * @param a11, a12, a21, a22  the values of affine matrix.
       * @param aTranslate  the 2D dimensional vector which represents translation.
       */
    ForwardAffineTransformation2D ( const double a11, const double a12, const double a21, const double a22, const RealVector & aTranslate )
    {
        BOOST_ASSERT((a11*a22!=a12*a21));
        this->origin = RealPoint(0,0);
        RealMatrix aMatrix;
        aMatrix(0,0) = a11;
        aMatrix(0,1) = a12;
        aMatrix(1,0) = a21;
        aMatrix(1,1) = a22;
        this->transform_matrix = aMatrix;
        this->translation = aTranslate;
    }
}; //end of class ForwardAffineTransformation2D

/////////////////////////////////////////////////////////////////////////////
// Template class BackwardAffineTransformation2D
/**
     * Description of template functor like class 'BackwardAffineTransformation2D' <p>
     * \brief Aim: implements backward rigid transformation of point in the 2D integer space.
     * Warring: This version uses closest neighbor interpolation.
     *
     * @tparam TSpace a 2 dimensional space.
     * @tparam TInputValue type of the input point e.g., TSpace::RealPoint
     * @tparam TOutputValue type of the output point e.g., TSpace::Point
     * @tparam TFunctor a functor operating on the output e.g., a rounding function.
     *
     * @see exampleAffineTransformation3d.cpp
     */
template < typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point,
           typename TFunctor = VectorRounding < TInputValue, TOutputValue > >
class BackwardAffineTransformation2D : public GeometricTransformation2D <TSpace,TInputValue,TOutputValue,TFunctor>
{
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( concepts::CSpace<TSpace> ));
    BOOST_STATIC_ASSERT(( TSpace::dimension == 2 ));
    BOOST_STATIC_ASSERT(( TOutputValue::dimension == 2 ));
    //BOOST_STATIC_ASSERT(( TInputValue::dimension == 2 ));

    // ----------------------- Types ------------------------------
public:
    typedef typename TSpace::RealPoint RealPoint;
    typedef typename TSpace::RealVector RealVector;
    typedef Eigen::Matrix<double, TSpace::dimension, TSpace::dimension> RealMatrix;

    // ----------------------- Interface --------------------------------------
public:
    /**
       * Constructor.
       * @param aOrigin  the center of affine transform.
       * @param aMatrix  the affine matrix.
       * @param aTranslate  the 2D dimensional vector which represents translation.
       */
    BackwardAffineTransformation2D ( const RealPoint& aOrigin, const RealMatrix & aMatrix, const RealVector & aTranslate )
    {
        this->origin = aOrigin;
        BOOST_ASSERT((aMatrix(0,0)*aMatrix(1,1)!=aMatrix(1,0)*aMatrix(0,1)));
        this->transform_matrix = aMatrix;
        this->translation = aTranslate;
    }

    /**
       * Constructor.
       * @param a11, a12, a21, a22  the values of affine matrix.
       * @param aTranslate  the 2D dimensional vector which represents translation.
       */
    BackwardAffineTransformation2D ( const double a11, const double a12, const double a21, const double a22, const RealVector & aTranslate )
    {
        BOOST_ASSERT((a11*a22!=a12*a21));
        this->origin = RealPoint(0,0);
        RealMatrix aMatrix;
        double det = a11*a22-a21*a12;
        aMatrix(0,0) = a22/det;
        aMatrix(0,1) = -a12/det;
        aMatrix(1,0) = -a21/det;
        aMatrix(1,1) = a11/det;
        this->transform_matrix = aMatrix;
        this->translation = aTranslate;
    }
    
    TOutputValue operator()( const TInputValue & aInput ) const override
    {
        RealPoint p;
        double a = this->transform_matrix(0,0);//transform_matrix.at(0).at(0);
        double b = this->transform_matrix(0,1);//transform_matrix.at(0).at(1);
        double c = this->transform_matrix(1,0);//transform_matrix.at(1).at(0);
        double d = this->transform_matrix(1,1);//transform_matrix.at(1).at(1);
        p[0] = ( a * ( aInput[0] - this->origin[0] - this->translation[0] ) +
               b * ( aInput[1] - this->origin[1] - this->translation[1] ) ) + this->origin[0];

        p[1] = ( c * ( aInput[0] - this->origin[0] - this->translation[0] ) +
               d * ( aInput[1] - this->origin[1] - this->translation[1] ) ) + this->origin[1];
        return this->functor ( p );
    }
};

}// namespace DGtal::functors
}// namespace DGtal

#endif // !defined AffineTransformation2D_h

#undef AffineTransformation2D_RECURSES
#endif // else defined(AffineTransformation2D_RECURSES)

