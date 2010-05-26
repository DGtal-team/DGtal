#pragma once

/**
 * @file BoxDomain.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/25
 *
 * Header file for module BoxDomain.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(BoxDomain_RECURSES)
#error Recursive header files inclusion detected in BoxDomain.h
#else // defined(BoxDomain_RECURSES)
/** Prevents recursive inclusion of headers. */
#define BoxDomain_RECURSES

#if !defined BoxDomain_h
/** Prevents repeated inclusion of headers. */
#define BoxDomain_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class BoxDomain
/**
 * Description of class 'BoxDomain' <p>
 * Aim:
 */
template<class TSpace>
class HyperRectDomain
{
    // ----------------------- Standard services ------------------------------
public:

    typedef typename TSpace::PointType PointType;
    typedef TSpace SpaceType;

    /**
    * Default Constructor.
    */
    HyperRectDomain();


    /**
    * Constructor from  two points \param aPointA and \param aPoint B
    * defining the space diagonal.
    *
    */
    HyperRectDomain(const PointType &aPointA, const PointType &aPointB);


    /**
     * Destructor.
     */
    ~HyperRectDomain();


    /**
    * ConstIterator class for BoxDomain
    *
    **/
    class ConstIterator {

        PointType myPoint;
        PointType mylower, myupper;
        std::size_t myCurrentDim;
    public:
        ConstIterator(  const PointType & p, const PointType& lower,const PointType &upper )
                : myPoint( p ), myCurrentDim(0), mylower(lower), myupper(upper)
        {}

        const PointType & operator*() const
        {
            return myPoint;
        }

        bool operator!=( const ConstIterator &aIt ) const
        {
            return (myPoint != (*aIt) );
        }

        void next()
        {
            if (myPoint.at(myCurrentDim) + 1 <= myupper.at(myCurrentDim))
                myPoint.at(myCurrentDim) ++;
            else
                if (myCurrentDim +1 < myPoint.dimension())
                {
                    myPoint.at(myCurrentDim) = mylower.at(myCurrentDim);
                    myPoint.at(myCurrentDim +1) ++;
                }
                else
                {
                    ///	      arg.. je renvoie une exception ?
                }
        }

        ConstIterator &operator++()
        {
            this->next();
            return *this;
        }
    };

    /**
    * begin() iterator.
    *
    **/
    ConstIterator begin() const;

    /**
    * end() iterator.
    *
    **/
    ConstIterator end() const;


    // ----------------------- Interface --------------------------------------
public:


    /**
    * Returns the lowest point of the space diagonal.
    *
    **/
    const PointType &lowerBound() const;

    /**
    * Returns the highest point of the space diagonal.
    *
    **/
    const PointType &upperBound() const ;


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

    PointType myLowerBound; ///The lowest point of the space diagonal
    PointType myUpperBound;///The highest point of the space diagonal

    // ------------------------- Hidden services ------------------------------
protected:



private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    HyperRectDomain ( const HyperRectDomain & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    HyperRectDomain & operator= ( const HyperRectDomain & other );

    // ------------------------- Internals ------------------------------------
private:

}; // end of class BoxDomain


/**
 * Overloads 'operator<<' for displaying objects of class 'BoxDomain'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'BoxDomain' to write.
 * @return the output stream after the writing.
 */
template<class TSpace>
std::ostream&
operator<< ( std::ostream & out, const HyperRectDomain<TSpace> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/kernel/HyperRectDomain.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined BoxDomain_h

#undef BoxDomain_RECURSES
#endif // else defined(BoxDomain_RECURSES)
