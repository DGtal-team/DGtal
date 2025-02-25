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
 * @file NaiveParametricCurveDigitizer3D.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, A3SI, France
 *
 * @date 2018/08/06
 *
 * Header file for module NaiveParametricCurveDigitizer3D
 *
 * This file is part of the DGtal library.
 */

#if defined(CurveDigitizer_RECURSES)
#error Recursive header files inclusion detected in CurveDigitizer.h
#else // defined(CurveDigitizer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CurveDigitizer_RECURSES

#if !defined CurveDigitizer_h
/** Prevents repeated inclusion of headers. */
#define CurveDigitizer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"
#include <vector>
#include <map>
#include <iterator>
#include "DGtal/geometry/curves/parametric/C3DParametricCurve.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class NaiveParametricCurveDigitizer3D
/**
 * Description of class 'NaiveParametricCurveDigitizer3D' <p>
 * \brief Aim: Digitization of 3D parametric curves.
 * This method produces, for good parameters step and k_next, a 26-connected
 * digital curves obtained from a digitization process of 3D parametric curves.
 *
 * @tparam TParametricCurve a model of C3DParametricCurve
 * 
 * \todo The method cannot detect an optimal digitization time step, therefore, for curves of high
 * curvature, the method can be very very slow due to the need of setting a tiny digitization step.
 * Such a limitation is planned to be mitigated by splitting a curve into regions of different
 * digitization step.
 *
 */
template <concepts::C3DParametricCurve TParametricCurve>
class NaiveParametricCurveDigitizer3D
{
    // ----------------------- Standard services ------------------------------
public:
    /// Integer point type
    typedef typename TParametricCurve::Space::Point Point;
    /// Real point type
    typedef typename TParametricCurve::Space::RealPoint RealPoint;
    /// Digital curve type
    typedef std::vector<Point> DigitalCurve;
    /**
     * MetaData type. The first field stores the time value at which the continuous curve is at the closest
     * position to the corresponding integer point. This time value is evaluated with respect to the parameter
     * timeStep (see documentation of the method NaiveParametricCurveDigitizer3D::init).
     * The second information stores the number of times a given integer point was hit during the digitization.
     */
    typedef std::vector< std::pair < long double, unsigned int > > MetaData;

    /**
     * Constructor
     */
    NaiveParametricCurveDigitizer3D();

    /**
     * Destructor.
     */
    ~NaiveParametricCurveDigitizer3D() = default;


    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    NaiveParametricCurveDigitizer3D ( const NaiveParametricCurveDigitizer3D & other ) = delete;


    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    NaiveParametricCurveDigitizer3D & operator= ( const NaiveParametricCurveDigitizer3D & other ) = delete;

    // ----------------------- Interface --------------------------------------
public:

    /**
     * @param p_curve - a paramtric curve realizing the model C3DParametricCurve
     */
    void attach ( ConstAlias<TParametricCurve> p_curve );

    /**
     * @param tmin - starting time (has to be lower than tmax)
     * @param tmax - the time when the digitization should stop (has to be bigger than tmin)
     * @param timeStep - the digitization step
     */
    void init ( long double tmin, long double tmax, long double timeStep );

    /**
     * @param knext - the parameter used for scanning k points of the curve in order to fix possible connectivity issues. The default value is 5.
     * @return previous value of K_NEXT
     */
    unsigned int setKNext ( unsigned int knext );


    /**
     * @param inserter writable iterator over a container which stores points of digitized curve
     */
    void digitize ( std::back_insert_iterator < DigitalCurve > inserter );

    /**
     * @param inserter writable iterator over a container which stores points of digitized curve
     * @param meta_inserter writable iterator over a container which stores meta information
     * (t at which curve is closest to the integer point (wwith respect to timeStep), hit values) used during digitization.
     */
    void digitize ( std::back_insert_iterator < DigitalCurve > inserter, std::back_insert_iterator < MetaData > meta_inserter );

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Private Data --------------------------------
private:

    /**
     * Buffer type. A buffer is used during digitzation to store part of the output. The size of the bufffer is
     * three times K_NEXT.
     */
    typedef DigitalCurve Buffer;

    /**
     * A type representing a const iterator over Buffer.
     */
    typedef typename Buffer::const_iterator ConstIterator;

    /**
     * Type which defines storage of meta data related to an integer point.
     * The final meta data required by the user are computed from the data stored in DataInfo.
     */
    typedef std::map < Point, std::pair < long double, unsigned int > > DataInfo;

    /**
     * This value is used during the local digitzation step. In general, we search for K_NEXT points and check if
     * they are 26-connected. Also, the internal buffer size is defined as three times K_NEXT. The default value is 5.
     */
    unsigned int K_NEXT;

    /**
     * A variable which stores the buffer size. It is equl to 3 times K_NEXT.
     */
    unsigned int BUFFER_SIZE;

    /// A structure used for making iterations over digital curve with respect to K_NEXT.
    struct KConstIter { typename DigitalCurve::const_iterator jt; unsigned int k; };
    /// A structure used for making iterations over digital curve with respect to K_NEXT.
    struct KIter { typename DigitalCurve::iterator jt; unsigned int k; };

    // ------------------------- Protected Data ------------------------------
protected:
    /// A storage of final integer points
    DigitalCurve digitalCurve;
   /// A storage of final meta data (if requested)
    MetaData metaDataContainter;
    /// A flag used to decided if meta data should be stored and returned to the user
    bool metaData;
    /// A pointer to the parameteric curve which is going to be digitized
    const TParametricCurve * curve;
    /// the time step value
    long double step;
    /// starting time (has to be lower than timeMax)
    long double timeMin;
    ///  the time when the digitization should stop (has to be bigger than timeMin)
    long double timeMax;
    /// A flag which is set to true if the initial paramters are correct.
    bool initOK;

    // ------------------------- Hidden services ------------------------------
private:

    /**
     * Checks is two points are 26-connected
     * @param x an integer point
     * @param y an integer point
     * @return true if x and y are 26-connected and false otherwise
     */
    bool is26Connected ( const Point &x, const Point &y );
    /**
     * A method used to synchronized the final data with the buffer
     * @param bbegin an iterator pointing at the beginning of the buffer
     * @param bend an iterator pointing at the end of the buffer
     * @param weights a reference to the meta information used to construct the final curve.
     */
    void syncData ( ConstIterator bbegin, ConstIterator bend, DataInfo & weights );

    /**
     *
     * @param buffer reference to the buffer to be flashed
     * @param weights removes a part of the data corresponding to the buffer points but keeps
     * part of the data that may still be needed.
     */
    void flashBuffers ( Buffer & buffer, DataInfo & weights );

    /**
     * Checks if a closer point of the continuous curve to the integer point has been found
     * and if so then the meta information are being updated.
     * @param p an integer point
     * @param pc a real point
     * @param weights meta data storage
     * @param t given time
     */
    void updateMetaData ( const Point & p, const RealPoint & pc, DataInfo & weights, long double t );

    /**
     * Final clean up at the ends of the curve`
     */
    void cleanCurve ( );

    /**
     * Final clean up of a closed curve at the possibly overlapping part.
     */
    void cleanClosedPart ( );



    // ------------------------- Internals ------------------------------------

}; // end of class CurveDigitizer


/**
 * Overloads 'operator<<' for displaying objects of class 'NaiveParametricCurveDigitizer3D'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'NaiveParametricCurveDigitizer3D' to write.
 * @return the output stream after the writing.
 */
template <typename T>
std::ostream&
operator<< ( std::ostream & out, const NaiveParametricCurveDigitizer3D<T> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/geometry/curves/parametric/NaiveParametricCurveDigitizer3D.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CurveDigitizer_h

#undef CurveDigitizer_RECURSES
#endif // else defined(CurveDigitizer_RECURSES)
