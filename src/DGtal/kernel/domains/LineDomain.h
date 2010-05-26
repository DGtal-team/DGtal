#pragma once

/**
 * @file LineDomain.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/26
 *
 * Header file for module LineDomain.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(LineDomain_RECURSES)
#error Recursive header files inclusion detected in LineDomain.h
#else // defined(LineDomain_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LineDomain_RECURSES

#if !defined LineDomain_h
/** Prevents repeated inclusion of headers. */
#define LineDomain_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class LineDomain
/**
 * Description of class 'LineDomain' <p>
 * Aim:
 */

template<class TSpace>
class LineDomain : public HyperRectDomain<TSpace>
{

public:

    typedef typename HyperRectDomain<TSpace>::PointType PointType;


    /**
    * Constructor.
    */
    LineDomain();


    /**
     * Destructor.
     */
    ~LineDomain();

    // ----------------------- Interface --------------------------------------
public:


    /**
    * ConstIterator class for HyperRectDomain.
    *
    **/
    class ConstIterator {

        ///Current Point in the domain
        PointType myPoint;
        ///Copies of the Domain limits
        PointType mylower, myupper;
        ///Second index of the iterator position
        std::size_t myCurrentPos;

    public:

        typedef std::bidirectional_iterator_tag iterator_category; ///\todo construct a RANDOM-ACCESS iterator
        typedef PointType value_type;
        typedef ptrdiff_t difference_type;
        typedef PointType* pointer;
        typedef PointType& reference;


        ConstIterator(  const PointType & p, const PointType& lower,const PointType &upper )
                : myPoint( p ),  myCurrentPos(0), mylower(lower), myupper(upper)
        {
        }

        const PointType & operator*() const
        {
            return myPoint;
        }

        /**
        * Operator ==
        *
        */
        bool operator==(const ConstIterator &it) const
        {
            return (myPoint == (*it));
        }

        /**
        * Operator !=
        *
        */
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
            if (myPoint.at(myCurrentPos)  < myupper.at(myCurrentPos))
                myPoint.at(myCurrentPos) ++;
            else
            {
                while ((myCurrentPos < myPoint.dimension()) &&
                        (myPoint.at(myCurrentPos)  >=  myupper.at(myCurrentPos)))
                {
                    myPoint.at(myCurrentPos) = mylower.at(myCurrentPos);
                    myCurrentPos++;
                }

                if  (myCurrentPos < myPoint.dimension())
                {
                    myPoint.at(myCurrentPos) ++;
                    myCurrentPos = 0;
                }
                else
                {
                    myPoint = myupper;
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


        /**
        * Implements the prev() method to scan the domain points dimension by dimension
        * (lexicographic order).
        *
        **/
        void prev()
        {
            if ( myPoint.at( myCurrentPos )  > mylower.at( myCurrentPos ) )
                myPoint.at( myCurrentPos ) --;
            else
            {
                while ((myCurrentPos >= 0) &&
                        (myPoint.at(myCurrentPos)  <=  mylower.at(myCurrentPos)))
                {
                    myPoint.at(myCurrentPos) = myupper.at(myCurrentPos);
                    myCurrentPos++;
                }

                if  ( myCurrentPos >= 0 )
                {
                    myPoint.at(myCurrentPos) --;
                    myCurrentPos = 0;
                }
                else
                {
                    myPoint = mylower;
                }
            }

        }

        /**
        * Operator ++ (++it)
        *
        */
        ConstIterator &operator--()
        {
            this->prev();
            return *this;
        }

        /**
        * Operator ++ (it++)
        *
        */
        ConstIterator &operator--(int)
        {
            ConstIterator tmp = *this;
            --*this;
            return tmp;
        }


    };


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



private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    LineDomain ( const LineDomain & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    LineDomain & operator= ( const LineDomain & other );

    // ------------------------- Internals ------------------------------------
private:

}; // end of class LineDomain


/**
 * Overloads 'operator<<' for displaying objects of class 'LineDomain'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'LineDomain' to write.
 * @return the output stream after the writing.
 */
template<class TSpace>
std::ostream&
operator<< ( std::ostream & out, const LineDomain<TSpace> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/kernel/domains/LineDomain.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LineDomain_h

#undef LineDomain_RECURSES
#endif // else defined(LineDomain_RECURSES)
