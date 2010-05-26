#pragma once

/**
 * @file HyperRectDomain.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/25
 *
 * Header file for module HyperRectDomain.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HyperRectDomain_RECURSES)
#error Recursive header files inclusion detected in HyperRectDomain.h
#else // defined(HyperRectDomain_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HyperRectDomain_RECURSES

#if !defined HyperRectDomain_h
/** Prevents repeated inclusion of headers. */
#define HyperRectDomain_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class HyperRectDomain
/**
 * Description of class 'HyperRectDomain' <p>
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
    * ConstIterator class for HyperRectDomain.
    *
    **/
    class ConstIterator {

        ///Current Point in the domain
        PointType myPoint;
        ///Copies of the Domain limits
        PointType mylower, myupper;
        ///Index of the iterator position
        std::size_t myCurrentDim;
        ///Second index of the iterator position
        std::size_t myCurrentPos;

    public:

        typedef std::forward_iterator_tag iterator_category; ///\todo construct a bidirectionnal iterator
        typedef PointType value_type;
        typedef ptrdiff_t difference_type;
        typedef PointType* pointer;
        typedef PointType& reference;


        ConstIterator(  const PointType & p, const PointType& lower,const PointType &upper )
                : myPoint( p ), myCurrentDim(0), myCurrentPos(0), mylower(lower), myupper(upper)
        {}

        const PointType & operator*() const
        {
            return myPoint;
        }

        bool operator!=( const ConstIterator &aIt ) const
        {
            return (myPoint != (*aIt) );
        }

        /**
        * Implements the next() method to scan the domain points dimension by dimension
        * (lexicographic order).
        *
        **/
        void next()
        {
            if (myPoint.at(myCurrentPos) + 1 <= myupper.at(myCurrentPos))
                myPoint.at(myCurrentPos) ++;
            else
            {
                if (myCurrentPos < myCurrentDim)
                {
                    myCurrentPos++;
                    myPoint.at(myCurrentPos) ++;
                }
                else
                {
                    if (myCurrentDim < myPoint.dimension())
                    {
                        for (unsigned int i=0 ; i <= myCurrentDim ; ++i)
                            myPoint.at(i) = mylower.at(i);
                        myPoint.at(myCurrentDim +1)++;

                        myCurrentPos = 0;
                        myCurrentDim++;
                    }
                }
            }
        }

        /**
        * Operator ++ (++it)
        *
        */
        ConstIterator &operator++()
        {
            this->next();
            return *this;
        }
        
        /**
	* Operator ++ (it++)
	*
	*/
        ConstIterator &operator++(int)
        {
            ConstIterator tmp = *this;
            ++*this;
            return tmp;
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

    ///The lowest point of the space diagonal
    PointType myLowerBound;
    ///The highest point of the space diagonal
    PointType myUpperBound;

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

}; // end of class HyperRectDomain


/**
 * Overloads 'operator<<' for displaying objects of class 'HyperRectDomain'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'HyperRectDomain' to write.
 * @return the output stream after the writing.
 */
template<class TSpace>
std::ostream&
operator<< ( std::ostream & out, const HyperRectDomain<TSpace> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/kernel/domains/HyperRectDomain.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HyperRectDomain_h

#undef HyperRectDomain_RECURSES
#endif // else defined(HyperRectDomain_RECURSES)
