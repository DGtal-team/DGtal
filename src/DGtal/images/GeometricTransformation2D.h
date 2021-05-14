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
 * @file GeometricTransformation2D.h
 * @author Phuc Ngo (\c hoai-diem-phuc.ngo@loria.fr )
 * Laboratoire Lorrain de Recherche en Informatique et ses Applications (LORIA), France
 *
 * @date 08/01/2021
 *
 * This file is part of the DGtal library.
 */

#if defined(GeometricTransformation2D_RECURSES)
#error Recursive header files inclusion detected in GeometricTransformation2D.h
#else // defined(GeometricTransformation2D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GeometricTransformation2D_RECURSES

#if !defined GeometricTransformation2D_h
/** Prevents repeated inclusion of headers. */
#define GeometricTransformation2D_h

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
#include <Eigen/Eigen>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
namespace functors
{
/////////////////////////////////////////////////////////////////////////////
// Template class GeometricTransformation2D
/**
     * Description of template functor like class 'GeometricTransformation2D' <p>
     * \brief Aim: implements geometric transformation of point in the 2D integer space.
     * This version uses closest neighbor interpolation.
     *
     * @tparam TSpace a 2 dimensional space.
     * @tparam TInputValue type of the input point e.g., TSpace::RealPoint
     * @tparam TOutputValue type of the output point e.g., TSpace::Point
     * @tparam TFunctor a functor operating on the output e.g., a rounding function.
     *
     * @see exampleAffinetransformation2d.cpp
     */
template < typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point,
           typename TFunctor = VectorRounding < TInputValue, TOutputValue > >
class GeometricTransformation2D
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
    // ------------------------- Protected Datas ------------------------------
    protected:
        TInputValue origin; //origin of transformation
        RealMatrix transform_matrix; //Transformation matrix
        RealVector translation; //Transmation vector
        TFunctor functor; //Interpolation function
    // ----------------------- Interface --------------------------------------
public:
    
    /**
       * Defaut Constructor without parameter
       */
    GeometricTransformation2D () { }
    /**
       * Constructor.
       * @param aOrigin  the center of rotation.
       * @param aMatrix  the transformation matrix.
       * @param aTranslate  the 2D dimensional vector which represents translation.
       */
    GeometricTransformation2D ( const RealPoint & aOrigin, const RealMatrix & aMatrix, const RealVector & aTranslate )
        : origin(aOrigin), translation(aTranslate) {
            transform_matrix(0,0)=aMatrix(0,0);
            transform_matrix(0,1)=aMatrix(0,1);
            transform_matrix(1,0)=aMatrix(1,0);
            transform_matrix(1,1)=aMatrix(1,1);
            //std::cout << "transform_matrix:\n" << transform_matrix << std::endl;
        }

    /**
       * Operator
       *
       * @return the transformed point.
       */
    inline
    virtual
    TOutputValue operator()( const TInputValue & aInput ) const
    {
        RealPoint p;
        double a = transform_matrix(0,0);//transform_matrix.at(0).at(0);
        double b = transform_matrix(0,1);//transform_matrix.at(0).at(1);
        double c = transform_matrix(1,0);//transform_matrix.at(1).at(0);
        double d = transform_matrix(1,1);//transform_matrix.at(1).at(1);
        p[0] = ( ( a * ( aInput[0] - origin[0] ) +
               b * ( aInput[1] - origin[1] ) ) + translation[0] ) + origin[0];

        p[1] = ( ( c * ( aInput[0] - origin[0] ) +
               d * ( aInput[1] - origin[1] ) ) + translation[1] ) + origin[1];
        return functor ( p );
    }

};

/////////////////////////////////////////////////////////////////////////////
// Template class DomainGeometricTransformation2D
/**
     * Description of template functor like class 'DomainGeometricTransformation2D' <p>
     * \brief Aim: implements bounds of transformed domain.
     *
     * @tparam TDomain a 2 dimensional domain.
     * @tparam TGeometricTransformFunctor a functor which represent two dimensional geometric transformation.
     *
     * @see FIXME: exampleRigidtransformation2d.cpp
     */
template <typename TDomain, typename TGeometricTransformFunctor >
class DomainGeometricTransformation2D
{
    ///Checking concepts
    BOOST_STATIC_ASSERT(( TDomain::dimension == 2 ));
    BOOST_CONCEPT_ASSERT(( concepts::CDomain<TDomain> ));

    // ----------------------- Types ------------------------------
public:
    typedef std::pair < typename TDomain::Space::Point, typename TDomain::Space::Point > Bounds;
    // ------------------------- Protected Datas ------------------------------
protected:
   const TGeometricTransformFunctor & transform;
    // ----------------------- Interface --------------------------------------
public:
    /**
       * Constructor.
       * @param aGeometricFunctor  - geometric transformation functor.
       */
    DomainGeometricTransformation2D ( const  TGeometricTransformFunctor & aFunctor ) : transform ( aFunctor ) {}

    /**
       * Operator
       *
       * @return bounds of the transformed domain.
       */
    inline
    Bounds operator()( const TDomain & aInput ) const
    {
        typedef typename TDomain::Point Point;
        Point points[4];
        points[0] = transform ( aInput.lowerBound() );
        points[1] = transform ( aInput.upperBound() );
        points[2] = transform ( Point ( aInput.upperBound()[0], aInput.lowerBound()[1] ) );
        points[3] = transform ( Point ( aInput.lowerBound()[0], aInput.upperBound()[1] ) );

        Point t_min ( INT_MAX, INT_MAX ), t_max ( INT_MIN, INT_MIN );
        for ( unsigned int i = 0; i < 4 ; i++ )
        {
            if ( points[i][0] < t_min[0] )
                t_min[0] = points[i][0];
            if ( points[i][1] < t_min[1] )
                t_min[1] = points[i][1];

            if ( points[i][0] > t_max[0] )
                t_max[0] = points[i][0];
            if ( points[i][1] > t_max[1] )
                t_max[1] = points[i][1];
        }

        Bounds bounds;
        bounds.first = t_min;
        bounds.second = t_max;
        return bounds;
    }
};

}// namespace DGtal::functors
}// namespace DGtal

#endif // !defined GeometricTransformation2D_h

#undef GeometricTransformation2D_RECURSES
#endif // else defined(GeometricTransformation2D_RECURSES)


