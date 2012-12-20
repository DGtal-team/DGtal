/**
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
**/

#pragma once

/**
* @file DGtalNode.h
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* INSTITUTION
*
* @date 2012/06/14
*
* Header file for module DGtalNode.cpp
*
* This file is part of the DGtal library.
*/
#if defined(DGtalSingleton_RECURSES)
#error Recursive header files inclusion detected in DGtalSingleton.h
#else // defined(DGtalSingleton_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DGtalSingleton_RECURSES

#if !defined DGtalSingleton_h
/** Prevents repeated inclusion of headers. */
#define DGtalSingleton_h

//////////////////////////////////////////////////////////////////////////////

// Includes

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
  {

	/**
	 * Description of struct 'Singleton' <p>
	 * \brief Ogre Factory for Singleton:
 	*/
    template <typename T> class Singleton
    {    


    public:
          // ----------------------- Standard services ------------------------------


          /**
          * Constructor 
          */
        Singleton( void )
        {
            assert( !mySingleton );
	    mySingleton = static_cast< T* >( this );
        }
	  /**
	  *  Destructor 
	  */
        ~Singleton( void )   
        {  
		assert( mySingleton );  
		mySingleton = 0;  
	}


	/**
	 * This function returns a reference on the unique object
	 */
        static T& getSingleton( void )
		
	{
		assert( mySingleton );  
		return ( *mySingleton );
	 }

	/**
	 * This function returns a pointer on the unique object
	 */
        static T* getSingletonPtr( void )		
	{
		 return mySingleton; 
	}

	protected:
          // ------------------------- Protected Datas ------------------------------
        static T* mySingleton;


	private:
          // ------------------------- Hidden services ------------------------------
          /**
          * Copy constructor.
          * @param other the object to clone.
          * Forbidden by default.
          */
	  Singleton(const Singleton<T> & other);

          /**
          * Copy constructor.
          * @param other the object to clone.
          * Forbidden by default.
          */
	Singleton& operator=(const Singleton<T> & other);
    };


}

///////////////////////////////////////////////////////////////////////////////


// //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DGtalSingleton_h
#undef DGtalSingleton_RECURSES
#endif // else defined(DGtalSingleton_RECURSES)




