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
 * @file
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2018/06/26
 *
 * Header file for module Parameters.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Parameters_RECURSES)
#error Recursive header files inclusion detected in Parameters.h
#else // defined(Parameters_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Parameters_RECURSES

#if !defined Parameters_h
/** Prevents repeated inclusion of headers. */
#define Parameters_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <sstream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /// Generic definition of a class for converting type X toward type Y.
  template <typename X, typename Y>
  struct ValueConverter {
    /// By default, it is impossible to do such conversions.
    static const Y& cast( const X& value ) const
    {
      BOOST_STATIC_ASSERT( false
			   && "[ValueConverter<X,Y>::cast] there is no such generic type converter." );
    }
  };
  
  /// Specialized definitions of a class for converting type X toward type Y.
  template <typename X>
  struct ValueConverter< X, X >{
    /// In this case, there is no conversion.
    static const X& cast( const X& value ) const
    { return value; }
  };
  
  /// Specialized definitions of a class for converting type X toward type Y.
  template <>
  struct ValueConverter< std::string, double >{
    static double cast( const std::string& value ) const
    { return atof( value ); }
  };
  
  /// Specialized definitions of a class for converting type X toward type Y.
  template <>
  struct ValueConverter< std::string, float >{
    static float cast( const std::string& value ) const
    { return (float) atof( value ); }
  };

  /// Specialized definitions of a class for converting type X toward type Y.
  template <>
  struct ValueConverter< std::string, int >{
    static int cast( const std::string& value ) const
    { return atoi( value ); }
  };
  /// Specialized definitions of a class for converting type X toward type Y.
  template < typename X >
  struct ValueConverter< X, std::string >{
    static std::string cast( const X& value ) const
    {
      std::ostringstream ss;
      ss << value;
      return ss.str();
    }
  };
      
    
  /// This class represents a parameter value as a string. It
  /// mimicks parameter values of boost::program_options.
  struct ParameterValue {
    typedef ParameterValue Self;
    ParameterValue ()                = default;
    ParameterValue ( const Self& v ) = default;
    ParameterValue ( Self&& v )      = default;
    Self& operator=( const Self& v ) = default;
    ParameterValue ( const std::string& v );
    template <typename X>
    ParameterValue ( const X& v );
    template <typename T>
    const T& as() const;
    void selfDisplay ( std::ostream & out ) const;
  protected:
    std::string _value;
  };
  
  /// This class represents a set of (input) simple parameters,
  /// i.e. mapping names to values.
  struct Parameters {
    /// The type of *this.
    typedef Parameters Self;
    
    /// Default constructor.
    Parameters()                         = default;
    /// Default destructor.
    ~Parameters()                        = default;
    /// Default copy constructor.
    Parameters( const Self& other )      = default;
    /// Default move.
    Parameters( Self&& other )           = default;
    /// Default assignment operator.
    Self& operator=( const Self& other ) = default;
    /// Constructor. Add parameter (name) or (name,pv)
    /// @param[in] name the name of the parameter.
    /// @param[in] pv the value of the parameter (string,int,float,double).
    Parameters( std::string name, ParameterValue pv = ParameterValue() );
    /// Add parameter (name) or (name,pv).
    /// @param[in] name the name of the parameter.
    /// @param[in] pv the value of the parameter (string,int,float,double).
    Self& operator()( std::string name, ParameterValue pv = ParameterValue() );
    /// @param[in] name any parameter name
    /// @param[in] unset the returned value if parameter is unset.
    /// @return the associated value (if it does not exist, return unset).
    ParameterValue operator[]( std::string name,
			       ParameterValue unset = "UNSET PARAMETER" ) const;
    /// @param[in] name any parameter name
    /// @return 'true' if and only if the parameter has been set or assigned.
    bool count( std::string name ) const;

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
  protected:
    std::map< std::string, ParameterValue > _parameters;
  };

  /**
   * Overloads 'operator<<' for displaying objects of class 'Parameters'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Parameters' to write.
   * @return the output stream after the writing.
   */
  std::ostream&
  operator<< ( std::ostream & out, const Parameters & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/helpers/Parameters.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Parameters_h

#undef Parameters_RECURSES
#endif // else defined(Parameters_RECURSES)
