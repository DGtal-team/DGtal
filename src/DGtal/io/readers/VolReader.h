#pragma once

/**
 * @file VolReader.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/25
 *
 * Header file for module VolReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(VolReader_RECURSES)
#error Recursive header files inclusion detected in VolReader.h
#else // defined(VolReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define VolReader_RECURSES

#if !defined VolReader_h
/** Prevents repeated inclusion of headers. */
#define VolReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <cstdio>
#include "DGtal/base/Common.h"
#include <boost/static_assert.hpp>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class VolReader
  /**
   * Description of template class 'VolReader' <p>
   * \brief Aim: implements methods to read a "Vol" file format.
   *
   * The main import methode "importVol" returns an instance of the template 
   * parameter TImageContainer.
   *
   * The private methods have been backported from the SimpleVol project 
   * (see http://liris.cnrs.fr/david.coeurjolly).
   *
   * Example usage:
   * @code
   * ...
   * typedef SpaceND<int,3> Space3;
   * typedef HyperRectDomain<Space3> TDomain;
   * typedef TDomain::Point Point;
   *
   * //Default image container = STLVector
   * typedef ImageSelector<TDomain, int>::Type Image;
   * 
   * VolReader<Image> reader;
   * Image image = reader.importVol("/home/dcoeurjo/Volumes/cat10.vol");
   *
   * trace.info() << image <<endl;
   * ...
   * @endcode
   *
   * @tparam TImageContainer the image container to use. 
   *
   * @example testVolReader.cpp
   */
  template <typename TImageContainer>
  class VolReader
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef TImageContainer ImageContainer;

    BOOST_STATIC_ASSERT(ImageContainer::Domain::staticDimension == 3);


    /** 
     * Default Constructor.
     * 
     */    
    VolReader();
    

    /** 
     * Default destructor.
     * 
     */
    ~VolReader();



    /** 
     * Main method to import a Vol into an instance of the 
     * template parameter ImageContainer.
     * 
     * @param filename the file name to import.
     * @return an instance of the ImageContainer.
     */
    ImageContainer & importVol(const std::string & filename);


    /** 
     * @return true if the object is valid.
     */
    bool isValid() 
    { 
      return myStateOk;
    }
    
    // ------------------------- Protected Datas ------------------------------
  private:

    typedef unsigned char voxel;

    //! Read vol data from a file already open
    ImageContainer *readVolData( FILE *in );
    
    //! Read raw data from a file already open
    ImageContainer *	readV1RawData( FILE *in, bool headerInited );
    
    //! Read raw data from a file already open
    ImageContainer * readV2RawData( FILE *in, bool headerInited, int sx, int sy, int sz );


    //! Returns NULL if this field is not found
    const char *getHeaderValue( const char *type ) const;

    //! Returns non-zero if failure
    int getHeaderValueAsInt( const char *type, int *dest ) const;

    //! This class help us to associate a field type and his value.
    //! An object is a pair (type, value). You can copy and assign
    //! such objects.
    /*! In recent C++, we should use a std::map, but we prefer (badly) code it
      by hand for compatibility with old compilers.
      At this time, there is a limit of 30 fields in header :-} */
    struct HeaderField {
      //! Constructor. The string are copied.
      HeaderField( const char *t, const char *v ) :
	type( strdup(t) ), value( strdup(v) ) {}
      ~HeaderField() {
	free( type );
	free( value );
      }
      //! Copy constructor
      HeaderField( const HeaderField &h ) :
	type( strdup(h.type) ), value( strdup(h.value) ) {};
      //! Default constructor
      HeaderField() : type(NULL), value(NULL) {};
      //! Assignement operator
      const HeaderField &operator = (const HeaderField &h) {
	free( type );
	free( value );
	if (h.type != NULL) {
	  type = strdup( h.type );
	  value = strdup( h.value );
	}
	return *this;
      }
      //! Type of field (e.g. Voxel-Size)
      char *type;
      //! Value of field (e.g. 2)
      char *value;
    };

    //! Maximum number of fields in a .vol file header
    static const int MAX_HEADERNUMLINES = 64;
    
    //! Here are the fields. A field is empty when his type string is NULL.
    HeaderField header[ MAX_HEADERNUMLINES ];
    
    //! Internal method which returns the index of a field or -1 if not found.
    int getHeaderField( const char *type ) const;
    
    //! Global list of required fields in a .vol file
    static const char *requiredHeaders[];
    
    
    //! A little structure that help us to determine host endian
    struct endian_t {
      //! Endian for the int type
      union {
	int 	i;
	char 	ci[ sizeof(int) + 1 ];
      } i_endian;

      //! Endian for the voxel type
      union {
	voxel	v;
	char 	cv[ sizeof(voxel) + 1 ];
      } v_endian;
    };

    endian_t endian;
    void initEndian();

    // ------------------------- Hidden services ------------------------------

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    VolReader ( const VolReader & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    VolReader & operator= ( const VolReader & other );

    // ------------------------- Internals ------------------------------------
  private:

    bool myStateOk;

  }; // end of class VolReader


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/VolReader.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined VolReader_h

#undef VolReader_RECURSES
#endif // else defined(VolReader_RECURSES)
