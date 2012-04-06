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
 * @file CCellularGridSpaceND.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/04/06
 *
 * Header file for concept CCellularGridSpaceND.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CCellularGridSpaceND_RECURSES)
#error Recursive header files inclusion detected in CCellularGridSpaceND.h
#else // defined(CCellularGridSpaceND_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CCellularGridSpaceND_RECURSES

#if !defined CCellularGridSpaceND_h
/** Prevents repeated inclusion of headers. */
#define CCellularGridSpaceND_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CCellularGridSpaceND
/**
Description of \b concept '\b CCellularGridSpaceND' <p>
@ingroup Concepts

@brief Aim: This concept describes a cellular grid space in nD. In
these spaces obtained by cartesian product, cells have a cubic shape
that depends on the dimension: 0-cells are points, 1-cells are unit
segments, 2-cells are squares, 3-cells are cubes, and so on.

Thsi concept is rather complex since it gathers all possible
operations on cells. The idea is that only the space knows what are
the cells, how to compute their adjacent or incident cells, how to
extract their coordinates, where are the bounds, what is the topology
and dimension of a cell, etc. Worse (!), you have two kinds of cells,
normal cells (unsigned), and oriented cells (signed). The latter are
used to define orientation and boundary operators.

@note Another way of defining orientation and boundary operators is to
define chains on cells and chain complexes. However, this is
unnecessary here.

Models of CCellularGridSpaceND are used whenever you need to define a
topology on your subsets of the digital space, especially boundaries
of objects, curves and digital surfaces.

The space is generally finite (except for arbitrary size
integers). The user should choose between a closed (default) cell
space or an open cell space.

### Refinement of

- boost::CopyConstructible

### Associated types :

- \e Integer: the integral type for representing coordinates in the space (model of CInteger).
- \e Size: the integral unsigned type for representing sizes in the space (model of CUnsignedInteger).
- \e Space: the corresponding digital space (same dimension and same \e Integer type as this).
- \e Cell: the type that represents an unsigned cell.
- \e SCell: the type that represents a signed cell.
- \e Surfel: the type that represents a signed n-1-cell. Should be promotable to SCell and reciprocally.
- \e Sign: the type that represents the sign/orientation of a cell. Should be promotable to \c bool and reciprocally (POS is true, NEG is false).
- \e DirIterator: the type that represents an iterator over the open or closed directions of a cell (signed or not) (model of boost::InputIterator).
- \e Point: the type for defining points in \e Space (same as Space::Point).
- \e Vector: the type for defining vectors in \e Space  (same as Space::Vector).
- \e Cells: a container that stores unsigned cells (not a set, rather a enumerable collection type, model of CConstSinglePassRange).
- \e SCells: a container that stores signed cells (not a set, rather a enumerable collection type, model of CConstSinglePassRange).
- \e CellSet: a set container that stores unsigned cells (efficient for queries like \c find, model of boost::UniqueAssociativeContainer and boost::SimpleAssociativeContainer).
- \e SCellSet: a set container that stores signed cells (efficient for queries like \c find, model of boost::UniqueAssociativeContainer and boost::SimpleAssociativeContainer).
- \e SurfelSet: a set container that stores surfels, i.e. signed n-1-cells (efficient for queries like \c find, model of boost::UniqueAssociativeContainer and boost::SimpleAssociativeContainer).
- \e CellMap<Value>: an associative container Cell->Value rebinder type (efficient for key queries). Use as \c typename X::template CellMap<Value>::Type, which is a model of boost::UniqueAssociativeContainer and boost::PairAssociativeContainer.
- \e SCellMap<Value>: an associative container SCell->Value rebinder type (efficient for key queries). Use as \c typename X::template SCellMap<Value>::Type, which is a model of boost::UniqueAssociativeContainer and boost::PairAssociativeContainer.
- \e SurfelMap<Value>: an associative container Surfel->Value rebinder type (efficient for key queries). Use as \c typename X::template SurfelMap<Value>::Type, which is a model of boost::UniqueAssociativeContainer and boost::PairAssociativeContainer.

### Notation
- \e X : A type that is a model of \e CCellularGridSpaceND
- \e x : object of type \e X
- \e k : object of type Dimension 
- \e i : object of type Integer
- \e c : object of type \e Cell 
- \e sc : object of type \e SCell 
- \e s : object of type \e Surfel 
- \e p, \e p1, \e p2 : object of type \e Point
- \e sign: object of type \e Sign

### Definitions

- a model of CCellularGridSpaceND is said \b closed if it includes the
  cells of lower dimension along its bounds. It is indeed closed in
  the sense of the star-topology.
- a model of CCellularGridSpaceND is said \b open if it does not include the
  cells of lower dimension along its bounds. It is indeed open in
  the sense of the star-topology.
- when it is initialized with points \e p1 and \e p2, the cellular
  space has a parallelepipedic shape bounded by the given
  coordinates. Any valid cell has then \e digital coordinates
  in-between \e p1 and \e p2 (included).
- \b digital coordinates are the natural coordinates of the cells of
  maximal dimension in the cellular space. For instance, it represents
  the coordinates of the pixels in an image. Two adjacent pixels have
  digital coordinates that differ by one in the cellular space. It
  is clear that incident cells (cells of lower dimension) may also
  have the same \e digital coordinates.
- \b Khalimsky coordinates are a way to number cells such that any two
  cells have different coordinates. More precisely, digital
  coordinates are doubled and the odd parity depends on whether or not
  the cell is open along this coordinate axis.
- a \b spel is a cell of maximal dimension (say n), a \b surfel is a
  cell of dimension n-1, a \b pointel is a cell of dimension 0.

### Valid expressions and semantics

| Name          | Expression       | Type requirements | Return type   | Precondition | Semantics                             | Post condition | Complexity |
|---------------|------------------|-------------------|---------------|--------------|---------------------------------------|----------------|------------|
| dimension     | \e x.dimension   |                   | \e Dimension  |              | the dimension of the space            |                |            |
| DIM           | \e x.dimension   |                   | \e Dimension  |              | the dimension of the space            |                |            |
| POS           | \e x.POS         |                   | \e Sign       |              | the positive sign for cells           |                |            |
| NEG           | \e x.NEG         |                   | \e Sign       |              | the negative sign for cells           |                |            |
|               |                  |                   |               |              |                                       |                |            |
| initialization|\e x.\e init(p1, p2, b)| b is \c bool | \c bool       |              | initializes the space so that cells are within the bounds p1 and p2, returns true iff the initialization was valid (ie, such bounds are representable with these integers).      |                |            |
| Size or width | \e.size( \e k )    |                 | \e Integer    |              | returns the size/width of the space along the axis \e k | |         |
| Minimal coordinate | \e.min( \e k )|                 | \e Integer    |              | returns the minimal possible digital coordinate along the axis \e k | | |
| Maximal coordinate | \e.max( \e k )|                 | \e Integer    |              | returns the maximal possible digital coordinate along the axis \e k | | |
| Lower bound   | \e x.lowerBound()|                   | \e Point      |              | returns the lowest point in the space, i.e. \e p1 |    |            |
| Upper bound   | \e x.upperBound()|                   | \e Point      |              | returns the uppermost point in the space, i.e. \e p1 | |            |
| Lower cell    | \e x.lowerCell() |                   | \e Cell       |              | returns the lowest cell in the space  |                |            |
| Upper cell    | \e x.upperCell() |                   | \e Cell       |              | returns the uppermost cell in the space|               |            |
| Closedness    | \e x.isSpaceClosed()|                | \c bool       |              | returns 'true' iff the cellular space is \e closed|    |            |
|               |                  |                   |               |              |                                       |                |            |
| Make unsigned cell | \e x.uCell(\e p)|               | \e Cell       |              | returns the unsigned cell with \e Khalimsky coordinates equal to \e p| | |
| Make unsigned cell | \e x.uCell(\e p, \e c)|         | \e Cell       |              | returns the unsigned cell with \e digital coordinates equal to \e p and topology equal to \e c| | |
| Make signed cell | \e x.sCell(\e p, \e sign = POS)|  | \e SCell      |              | returns the signed cell with \e Khalimsky coordinates equal to \e p and sign \e sign| | |
| Make signed cell | \e x.sCell(\e p, \e sc)|          | \e SCell      |              | returns the signed cell with \e digital coordinates equal to \e p and sign and topology equal to \e sc| | |
| Make unsigned spel | \e x.uSpel(\e p)|               | \e Cell       |              | returns the unsigned spel with \e digital coordinates equal to \e p| | |
| Make signed spel | \e x.sSpel(\e p, \e sign = POS)|  | \e SCell      |              | returns the signed spel with \e digital coordinates equal to \e p and sign \e sign| | |
| Make unsigned pointel | \e x.uPointel(\e p)|         | \e Cell       |              | returns the unsigned pointel with \e digital coordinates equal to \e p| | |
| Make signed pointel | \e x.sPointel(\e p, \e sign = POS)| | \e SCell |              | returns the signed pointel with \e digital coordinates equal to \e p and sign \e sign| | |
|               |                  |                   |               |              |                                       |                |            |
| Get Khalimsky coordinate| \e x.uKCoord(\e c, \e k)|  | \e Integer    |              | returns the Khalimsky coordinate of cell \e c along axis \e k | |   |
| Get digital coordinate| \e x.uCoord(\e c, \e k)|     | \e Integer    |              | returns the digital coordinate of cell \e c along axis \e k | |     |
| Get Khalimsky coordinates| \e x.uKCoords(\e c, \e k)| | \e Point     |              | returns the Khalimsky coordinates of cell \e c |       |            |
| Get digital coordinates| \e x.uCoords(\e c, \e k)|   | \e Point      |              | returns the digital coordinates of cell \e c |         |            |
| Get Khalimsky coordinate| \e x.sKCoord(\e sc, \e k)|  | \e Integer   |              | returns the Khalimsky coordinate of signed cell \e sc along axis \e k | | |
| Get digital coordinate| \e x.sCoord(\e sc, \e k)|     | \e Integer   |              | returns the digital coordinate of signed cell \e sc along axis \e k | | |
| Get Khalimsky coordinates| \e x.sKCoords(\e sc, \e k)| | \e Point    |              | returns the Khalimsky coordinates of signed cell \e sc |       |    |
| Get digital coordinates| \e x.sCoords(\e sc, \e k)|   | \e Point     |              | returns the digital coordinates of signed cell \e sc |         |    |
|               |                  |                   |               |              |                                       |                |            |
| Set Khalimsky coordinate |\e x.uSetKCoord(\e c,\e k,\e i)| |         |              | Sets the \e k-th Khalimsky coordinate of \e c to \e i | |           |
| Set digital coordinate |\e x.uSetCoord(\e c,\e k,\e i)| |            |              | Sets the \e k-th digital coordinate of \e c to \e i |  |            |
| Set Khalimsky coordinates |\e x.uSetKCoords(\e c,\e p)| |            |              | Sets the Khalimsky coordinates of \e c to \e p |       |            |
| Set digital coordinates |\e x.uSetCoords(\e c,\e p)| |               |              | Sets the digital coordinates of \e c to \e p |         |            |
| Set Khalimsky coordinate |\e x.sSetKCoord(\e sc,\e k,\e i)| |        |              | Sets the \e k-th Khalimsky coordinate of \e sc to \e i | |           |
| Set digital coordinate |\e x.sSetCoord(\e sc,\e k,\e i)| |           |              | Sets the \e k-th digital coordinate of \e sc to \e i |  |            |
| Set Khalimsky coordinates |\e x.sSetKCoords(\e sc,\e p)| |           |              | Sets the Khalimsky coordinates of \e sc to \e p |       |            |
| Set digital coordinates |\e x.sSetCoords(\e sc,\e p)| |              |              | Sets the digital coordinates of \e sc to \e p |         |            |
|               |                  |                   |               |              |                                       |                |            |
| Sign/orient cell | \e x.signs(\e c,\e sign)|         |               |              | returns the signed cell with same topology as \e c and sign \e sign| | |
| Change sign   | \e x.sSetSign(\e sc, \e sign)|       |               |              | Sets the sign of the signed cell \e sc to \e sign |    |            |
| Get sign      | \e x.sSign(\e sc)|                   | \e Sign       |              | returns the sign of cell \e sc        |                |            |
| Unsign/unorient signed cell | \e x.unsigns(\e sc)|   |               |              | returns the unsigned cell with same topology as \e sc| | |
| Flip sign     | \e x.sOpp(\e sc) |                   |               |              | returns the signed cell with opposite sign to \e sc| | |
|               |                  |                   |               |              |                                       |                |            |
|               |                  |                   |               |              |                                       |                |            |

### Invariants

### Models

- KhalimskySpaceND

### Notes

@tparam T the type that should be a model of CCellularGridSpaceND.
 */
template <typename T>
struct CCellularGridSpaceND 
  : boost::CopyConstructible<T>
{
  // ----------------------- Concept checks ------------------------------
public:
  typedef typename T::Integer Integer;
  typedef typename T::Size Size;
  typedef typename T::Space Space;
  typedef typename T::Cell Cell;
  typedef typename T::SCell SCell;
  typedef typename T::Surfel Surfel;
  typedef typename T::Sign Sign;
  typedef typename T::DirIterator DirIterator;
  typedef typename T::Point Point;
  typedef typename T::Vector Vector;
  typedef typename T::Cells Cells;
  typedef typename T::SCells SCells;
  typedef typename T::CellSet CellSet;
  typedef typename T::SCellSet SCellSet;
  typedef typename T::SurfelSet SurfelSet;
  typedef int Dummy;
  typedef typename T::template CellMap<Dummy>::Type CellMap;
  typedef typename T::template SCellMap<Dummy>::Type SCellMap;
  typedef typename T::template SurfelMap<Dummy>::Type SurfelMap;
  
  BOOST_CONCEPT_ASSERT(( CInteger< Integer > ));
  BOOST_CONCEPT_ASSERT(( CUnsignedInteger< Size > ));
  BOOST_STATIC_ASSERT(( ConceptUtils::SameType< Integer, typename Space::Integer >::value ));
  BOOST_STATIC_ASSERT(( ConceptUtils::SameType< Point, typename Space::Point >::value ));
  BOOST_STATIC_ASSERT(( ConceptUtils::SameType< Vector, typename Space::Vector >::value ));
  BOOST_CONCEPT_ASSERT(( CConstSinglePassRange< Cells > ));
  BOOST_CONCEPT_ASSERT(( CConstSinglePassRange< SCells > ));
  BOOST_CONCEPT_ASSERT(( boost::UniqueAssociativeContainer< CellSet > ));
  BOOST_CONCEPT_ASSERT(( boost::UniqueAssociativeContainer< SCellSet > ));
  BOOST_CONCEPT_ASSERT(( boost::UniqueAssociativeContainer< SurfelSet > ));
  BOOST_CONCEPT_ASSERT(( boost::SimpleAssociativeContainer< CellSet > ));
  BOOST_CONCEPT_ASSERT(( boost::SimpleAssociativeContainer< SCellSet > ));
  BOOST_CONCEPT_ASSERT(( boost::SimpleAssociativeContainer< SurfelSet > ));
  BOOST_CONCEPT_ASSERT(( boost::UniqueAssociativeContainer< CellMap > ));
  BOOST_CONCEPT_ASSERT(( boost::UniqueAssociativeContainer< SCellMap > ));
  BOOST_CONCEPT_ASSERT(( boost::UniqueAssociativeContainer< SurfelMap > ));
  BOOST_CONCEPT_ASSERT(( boost::PairAssociativeContainer< CellMap > ));
  BOOST_CONCEPT_ASSERT(( boost::PairAssociativeContainer< SCellMap > ));
  BOOST_CONCEPT_ASSERT(( boost::PairAssociativeContainer< SurfelMap > ));

  BOOST_CONCEPT_USAGE( CCellularGridSpaceND )
  {
    // ConceptUtils::sameType( myA, T::staticMember );
    ConceptUtils::sameType( myBool, myX.init( myP1, myP2, myBool ) );
    checkConstConstraints();
  }
  void checkConstConstraints() const
  {
    ConceptUtils::sameType( mySize, myX.size( myDim ) );
    ConceptUtils::sameType( myInteger, myX.min( myDim ) );
    ConceptUtils::sameType( myInteger, myX.max( myDim ) );
    ConceptUtils::sameType( myP1, myX.lowerBound() );
    ConceptUtils::sameType( myP2, myX.upperBound() );
    ConceptUtils::sameType( myCell, myX.lowerCell() );
    ConceptUtils::sameType( myCell, myX.upperCell() );
    ConceptUtils::sameType( myBool, myX.isSpaceClosed() );
    ConceptUtils::sameType( myCell, myX.uCell( myP1 ) );
    ConceptUtils::sameType( myCell, myX.uCell( myP1, myCell ) );
    ConceptUtils::sameType( mySCell, myX.sCell( myP1 ) );
    ConceptUtils::sameType( mySCell, myX.sCell( myP1, mySign ) );
    ConceptUtils::sameType( mySCell, myX.sCell( myP1, mySCell ) );
    ConceptUtils::sameType( myCell, myX.uSpel( myP1 ) );
    ConceptUtils::sameType( mySCell, myX.sSpel( myP1 ) );
    ConceptUtils::sameType( mySCell, myX.sSpel( myP1, myBool ) );
    ConceptUtils::sameType( myCell, myX.uPointel( myP1 ) );
    ConceptUtils::sameType( mySCell, myX.sPointel( myP1 ) );
    ConceptUtils::sameType( mySCell, myX.sPointel( myP1, myBool ) );
    ConceptUtils::sameType( myInteger, myX.uKCoord( myCell, myDim ) );
    ConceptUtils::sameType( myInteger, myX.uCoord( myCell, myDim ) );
    ConceptUtils::sameType( myP1, myX.uKCoords( myCell ) );
    ConceptUtils::sameType( myP1, myX.uCoords( myCell ) );
    ConceptUtils::sameType( myInteger, myX.sKCoord( mySCell, myDim ) );
    ConceptUtils::sameType( myInteger, myX.sCoord( mySCell, myDim ) );
    ConceptUtils::sameType( myP1, myX.sKCoords( mySCell ) );
    ConceptUtils::sameType( myP1, myX.sCoords( mySCell ) );
    myX.uSetKCoord( myMutableCell, myDim, myInteger );
    myX.uSetCoord( myMutableCell, myDim, myInteger );
    myX.uSetKCoords( myMutableCell, myP1 );
    myX.uSetCoords( myMutableCell, myP1 );
    myX.sSetKCoord( myMutableSCell, myDim, myInteger );
    myX.sSetCoord( myMutableSCell, myDim, myInteger );
    myX.sSetKCoords( myMutableSCell, myP1 );
    myX.sSetCoords( myMutableSCell, myP1 );
    ConceptUtils::sameType( mySign, myX.sSign( mySCell ) );
    myX.sSetSign( myMutableSCell, mySign );
    ConceptUtils::sameType( mySCell, myX.signs( myCell, mySign ) );
    ConceptUtils::sameType( mySCell, myX.sOpp( mySCell ) );
    ConceptUtils::sameType( myCell, myX.unsigns( mySCell ) );

  }
  // ------------------------- Private Datas --------------------------------
private:
  T myX; // do not require T to be default constructible.
  Integer myInteger;
  Size mySize;
  Dimension myDim;
  Point myP1, myP2;
  Cell myCell;
  SCell mySCell;
  mutable Cell myMutableCell;
  mutable SCell myMutableSCell;
  bool myBool;
  Sign mySign;

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CCellularGridSpaceND

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CCellularGridSpaceND_h

#undef CCellularGridSpaceND_RECURSES
#endif // else defined(CCellularGridSpaceND_RECURSES)
