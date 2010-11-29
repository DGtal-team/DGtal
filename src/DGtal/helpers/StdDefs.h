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
 * @file StdDefs.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/11/01
 *
 * Header file for module StdDefs.cpp.
 * Gathers standard definitions of digital spaces, adjacencies, topologies.
 *
 * This file is part of the DGtal library.
 */

#if defined(StdDefs_RECURSES)
#error Recursive header files inclusion detected in StdDefs.h
#else // defined(StdDefs_RECURSES)
/** Prevents recursive inclusion of headers. */
#define StdDefs_RECURSES

#if !defined StdDefs_h
/** Prevents repeated inclusion of headers. */
#define StdDefs_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/sets/DigitalSetSelector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/geometry/nd/volumetric/SeparableMetricTraits.h"
#include "DGtal/topology/DigitalTopology.h"
#include "DGtal/topology/MetricAdjacency.h"
#include "DGtal/topology/Object.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /**
   * This namespace gathers all standard type definitions as well as
   * some static instances for using a 2D digital space with integers
   * represented by 'int'. It is useful for a developer who wants to
   * develop an application in the 2D plane and who does not wish to
   * tune specifically the possible types.
   *
   * This is the standard set of types for 2D imagery.
   *
   * @todo Once stabilized, provide a Z2l (for int64) and a Z2I (for
   * arbitrary integers).
   */
  namespace Z2i {
    typedef SpaceND<2> Space;
    typedef Space Z2;
    /** Standard 4-neighborhood for a 2D digital space. */
    typedef MetricAdjacency< Space, 1> Adj4;
    /** Standard 8-neighborhood for a 2D digital space. */
    typedef MetricAdjacency< Space, 2> Adj8;
    typedef DigitalTopology< Adj4, Adj8 > DT4_8;
    typedef DigitalTopology< Adj8, Adj4 > DT8_4;
    typedef Space::Point Point;
    typedef Space::Vector Vector;
    typedef HyperRectDomain< Space > Domain; 
    typedef DigitalSetSelector< Domain, BIG_DS+HIGH_BEL_DS >::Type DigitalSet;
    typedef Object<DT4_8, DigitalSet> Object4_8;
    typedef Object<DT4_8, DigitalSet>::ComplementObject ComplementObject4_8;
    typedef Object<DT4_8, DigitalSet>::SmallObject SmallObject4_8;
    typedef Object<DT4_8, DigitalSet>::SmallComplementObject SmallComplementObject4_8;
    typedef Object<DT8_4, DigitalSet> Object8_4;
    typedef Object<DT8_4, DigitalSet>::ComplementObject ComplementObject8_4;
    typedef Object<DT8_4, DigitalSet>::SmallObject SmallObject8_4;
    typedef Object<DT8_4, DigitalSet>::SmallComplementObject SmallComplementObject8_4;

    typedef SeparableMetricTraits<DGtal::int32_t, DGtal::uint64_t,2> L2Metric;
    typedef SeparableMetricTraits<DGtal::int32_t, DGtal::int32_t,1> L1Metric;
    typedef SeparableMetricTraits<DGtal::int32_t, DGtal::uint32_t,0> LinfMetric;

    static const Adj4 adj4;
    static const Adj8 adj8;
    static const DT4_8 dt4_8 = DT4_8( adj4, adj8, JORDAN_DT );
    static const DT8_4 dt8_4 = DT8_4( adj8, adj4, JORDAN_DT );
  } // namespace Z2i

  /**
   * This namespacef gathers all standard type definitions as well as
   * some static instances for using a 3D digital space with integers
   * represented by 'int'. It is useful for a developer who wants to
   * develop an application in the 3D plane and who does not wish to
   * tune specifically the possible types.
   *
   * This is the standard set of types for 3D imagery.
   *
   * @todo Once stabilized, provide a Z3l (for int64) and a Z3I (for
   * arbitrary integers).
   */
  namespace Z3i {
    typedef SpaceND<3> Space;
    typedef Space Z3;
    /** Standard 6-neighborhood for a 3D digital space. */
    typedef MetricAdjacency< Space, 1> Adj6;
    /** Standard 18-neighborhood for a 3D digital space. */
    typedef MetricAdjacency< Space, 2> Adj18;
    /** Standard 26-neighborhood for a 3D digital space. */
    typedef MetricAdjacency< Space, 3> Adj26;
    typedef DigitalTopology< Adj6, Adj18 > DT6_18;
    typedef DigitalTopology< Adj18, Adj6 > DT18_6;
    typedef DigitalTopology< Adj6, Adj26 > DT6_26;
    typedef DigitalTopology< Adj26, Adj6 > DT26_6;
    typedef Space::Point Point;
    typedef Space::Vector Vector;
    typedef HyperRectDomain< Space > Domain; 
    typedef DigitalSetSelector< Domain, BIG_DS+HIGH_BEL_DS >::Type DigitalSet;
    typedef Object<DT6_18, DigitalSet> Object6_18;
    typedef Object<DT6_18, DigitalSet>::ComplementObject ComplementObject6_18;
    typedef Object<DT6_18, DigitalSet>::SmallObject SmallObject6_18;
    typedef Object<DT6_18, DigitalSet>::SmallComplementObject SmallComplementObject6_18;
    typedef Object<DT18_6, DigitalSet> Object18_6;
    typedef Object<DT18_6, DigitalSet>::ComplementObject ComplementObject18_6;
    typedef Object<DT18_6, DigitalSet>::SmallObject SmallObject18_6;
    typedef Object<DT18_6, DigitalSet>::SmallComplementObject SmallComplementObject18_6;
    typedef Object<DT6_26, DigitalSet> Object6_26;
    typedef Object<DT6_26, DigitalSet>::ComplementObject ComplementObject6_26;
    typedef Object<DT6_26, DigitalSet>::SmallObject SmallObject6_26;
    typedef Object<DT6_26, DigitalSet>::SmallComplementObject SmallComplementObject6_26;
    typedef Object<DT26_6, DigitalSet> Object26_6;
    typedef Object<DT26_6, DigitalSet>::ComplementObject ComplementObject26_6;
    typedef Object<DT26_6, DigitalSet>::SmallObject SmallObject26_6;
    typedef Object<DT26_6, DigitalSet>::SmallComplementObject SmallComplementObject26_6;

    typedef SeparableMetricTraits<Z3::Size, DGtal::uint64_t,2> L2Metric;
    typedef SeparableMetricTraits<Z3::Size, DGtal::int64_t,1> L1Metric;
    typedef SeparableMetricTraits<Z3::Size, DGtal::int64_t,0> LinfMetric;

    static const Adj6 adj6;
    static const Adj18 adj18;
    static const Adj26 adj26;
    static const DT6_18 dt6_18 = DT6_18( adj6, adj18, JORDAN_DT );
    static const DT18_6 dt18_6 = DT18_6( adj18, adj6, JORDAN_DT );
    static const DT6_26 dt6_26 = DT6_26( adj6, adj26, JORDAN_DT );
    static const DT26_6 dt26_6 = DT26_6( adj26, adj6, JORDAN_DT );
  } // namespace Z3i

  
/////////////////////////////////////////////////////////////////////////////
// class StdDefs
/**
 * Description of class 'StdDefs' <p>
 * \brief Aim:
 */
class StdDefs
{
    // ----------------------- Standard services ------------------------------
public:

    /**
     * Destructor.
     */
    ~StdDefs();

    // ----------------------- Interface --------------------------------------
public:

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

    // ------------------------- Protected Datas ------------------------------
private:
    // ------------------------- Private Datas --------------------------------
private:

    // ------------------------- Hidden services ------------------------------
protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    StdDefs();

private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    StdDefs ( const StdDefs & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    StdDefs & operator= ( const StdDefs & other );

    // ------------------------- Internals ------------------------------------
private:

}; // end of class StdDefs


/**
 * Overloads 'operator<<' for displaying objects of class 'StdDefs'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'StdDefs' to write.
 * @return the output stream after the writing.
 */
std::ostream&
operator<< ( std::ostream & out, const StdDefs & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/helpers/StdDefs.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined StdDefs_h

#undef StdDefs_RECURSES
#endif // else defined(StdDefs_RECURSES)
