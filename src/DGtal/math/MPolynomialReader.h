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
 * @file MPolynomialReader.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/07/06
 *
 * Header file for module MPolynomialReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(MPolynomialReader_RECURSES)
#error Recursive header files inclusion detected in MPolynomialReader.h
#else // defined(MPolynomialReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define MPolynomialReader_RECURSES

#if !defined MPolynomialReader_h
/** Prevents repeated inclusion of headers. */
#define MPolynomialReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/MPolynomial.h"
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/variant/recursive_variant.hpp>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  namespace detail {

    /**
       Part of the polynomial tree structure. 
       Terminal X_k^e
    */
    struct power_node {
      int k;
      int e;
    };

    /**
       Part of the polynomial tree structure. 
       Monomial coef ( X_k^e )*
    */
    struct monomial_node {
      double coef;
      std::vector<power_node> powers;
    };

    /**
       Part of the polynomial tree structure. 
       Sum of monomials
    */
    struct polynomial_node {
      std::vector<monomial_node> monomials;
    };

    struct top_node;

    /**
       Part of the polynomial tree structure. 
       Either a top_node or a polynomial.
    */
    typedef 
    boost::variant< boost::recursive_wrapper<top_node>, polynomial_node  >
    expr_node;

    /**
       Part of the polynomial tree structure.  An operation
       (+,-,*,^,()), subexpressions, and possibly the exponent value.
    */
    struct top_node {
      std::string op;
      std::vector<expr_node> expressions;
      int exp;
    };

  }

}

BOOST_FUSION_ADAPT_STRUCT(
                          DGtal::detail::power_node,
                          (int, k)
                          (int, e)
)
BOOST_FUSION_ADAPT_STRUCT(
                          DGtal::detail::monomial_node,
                          (double, coef)
                          (std::vector<DGtal::detail::power_node>, powers)
)
BOOST_FUSION_ADAPT_STRUCT(
                          DGtal::detail::polynomial_node,
                          (std::vector<DGtal::detail::monomial_node>, monomials)
)
BOOST_FUSION_ADAPT_STRUCT(
    DGtal::detail::top_node,
    (std::string, op)
    (std::vector<DGtal::detail::expr_node>, expressions)
    (int, exp)
)

namespace DGtal
{
  namespace qi = boost::spirit::qi;
  namespace ascii = boost::spirit::ascii;
  namespace phoenix = boost::phoenix;

  template <typename Iterator>
  struct MPolynomialGrammar 
    : qi::grammar<Iterator, detail::top_node(), ascii::space_type>
  {
    MPolynomialGrammar()
      : MPolynomialGrammar::base_type(top)
    {
      using qi::eps;
      using qi::lit;
      using qi::int_;
      using qi::_val;
      using qi::_1;
      using qi::double_;
      using phoenix::at_c;
      using phoenix::push_back;

      top = // sum of polynomials
        mulexpr [push_back(at_c<1>(_val), _1)] 
        >> ( ( lit('+') [ at_c<0>(_val) = "+" ] 
               >> top [push_back(at_c<1>(_val), _1)]  )
             | ( ( lit('-') [ at_c<0>(_val) = "-" ] 
                   >> top [push_back(at_c<1>(_val), _1)]  ) )
             | eps [ at_c<0>(_val) = "T" ] );

      mulexpr = // product of polynomials
        subexpr [push_back(at_c<1>(_val), _1)] 
        >> ( ( lit('*') [ at_c<0>(_val) = "*" ] 
               >> mulexpr [push_back(at_c<1>(_val), _1)] )
             | eps [ at_c<0>(_val) = "T" ] );

      subexpr =  // polynomial or parenthesis expr.
        mpolynomial
        | expexpr;
      expexpr = // ( expr ) or ( expr )^k
        ( lit('(') [ at_c<0>(_val) = "()" ] 
          >> top [ push_back(at_c<1>(_val), _1) ]
          >> lit(')') >> ( ( lit('^') >> int_ [ at_c<2>(_val) = _1 ] )
                           | eps [ at_c<2>(_val) = 1 ]  ) );
        
      mpolynomial = // monomial or sum of monomials
        monomial [push_back(at_c<0>(_val), _1)]
        >> *( lit('+') >> monomial [push_back(at_c<0>(_val), _1)] );
      monomial =   // coef * power(s)*, or power(s)+
        ( double_ [at_c<0>(_val) = _1] 
          >> *( genvariable [push_back(at_c<1>(_val), _1)] ) )
        | +( genvariable [push_back(at_c<1>(_val), _1)] 
             >> eps [at_c<0>(_val) = 1] );

      genvariable = litvariable | variable;
      variable = // X_0 X_1 ... X_0^4 X_1^2 X_3^0 ...
        lit('X') >> lit('_') 
                 >> int_ [at_c<0>(_val) = _1]
                 >> ( ( lit('^') >> int_ [at_c<1>(_val) = _1] ) // X_k^e
                      | eps [at_c<1>(_val) = 1] // X_k
                      );
      litvariable = // x y z t x^4 y^5 z^2 ...
        ( lit('x') [at_c<0>(_val) = 0]
          | lit('y') [at_c<0>(_val) = 1] 
          | lit('z') [at_c<0>(_val) = 2] 
          | lit('t') [at_c<0>(_val) = 3] )
        >> ( ( lit('^') >> int_ [at_c<1>(_val) = _1] ) // x^3 z^4
             | eps [at_c<1>(_val) = 1] // x y z
             );
    }
    qi::rule<Iterator, detail::top_node(), ascii::space_type> top;
    qi::rule<Iterator, detail::top_node(), ascii::space_type> mulexpr;
    qi::rule<Iterator, detail::expr_node(), ascii::space_type> subexpr;
    qi::rule<Iterator, detail::top_node(), ascii::space_type> expexpr;
    qi::rule<Iterator, detail::polynomial_node(), ascii::space_type> mpolynomial;
    qi::rule<Iterator, detail::monomial_node(), ascii::space_type> monomial;
    qi::rule<Iterator, detail::power_node(), ascii::space_type> variable;
    qi::rule<Iterator, detail::power_node(), ascii::space_type> genvariable;
    qi::rule<Iterator, detail::power_node(), ascii::space_type> litvariable;

  };
}

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class MPolynomialReader
  /**
     Description of template class 'MPolynomialReader' <p> 

     \brief Aim: This class converts a string polynomial expression in
     a multivariate polynomial.

     @code
     typedef double Ring;
     MPolynomial<3,Ring,std::allocator<Ring> > P;
     MPolynomialReader<3,Ring> reader;
     string s1 = "1.5 X_0^2 X_2^3 X_1^5 * (4 X_0^3 + X_1^2)^2";
     bool ok1 = reader.read( P, s1.begin(), s1.end() ) == s1.end();
     std::cout << "- Parsing " << s1 << " : " << ok1 << " " << P << std::endl;
     @encode
     
     @tparam n the number of variables or indeterminates.

     @tparam TRing the type chosen for the polynomial, defines also
     the type of the coefficents (generally int, float or double).
     
     @tparam TIterator the type chosen for iterating over characters.
  */
  template <int n, typename TRing, 
            typename TIterator = std::string::const_iterator>
  class MPolynomialReader
  {
  public:
    typedef TRing Ring;
    typedef TIterator Iterator;
    typedef MPolynomial<n, Ring, std::allocator<Ring> > Polynomial;
    typedef MPolynomialGrammar<Iterator> Grammar;
    
    /// Polynomial grammar.
    Grammar gpolynomial;
    
    /**
       Instantiates the grammar.
    */
    MPolynomialReader() {}

    /**
       Read any string between \a begin and \end, and builds the
       corresponding polynomial in \a p.

       @param p (returns) the polynomial
       @param begin an iterator on the first character to parse.
       @param end an iterator pointing after the last character to parse.
       @return an iterator on the last successfully read position.
    */
    Iterator read( Polynomial & p, Iterator begin, Iterator end )
    {
      using qi::phrase_parse;
      using ascii::space;
      detail::top_node m;
      bool r = phrase_parse( begin, end, gpolynomial, space, m );
      if (r) p = make( m );
      return r ? begin : end;
    }



    // ----------------------- Interface --------------------------------------
  public:

    /**
       Writes/Displays the object on an output stream.
       @param out the output stream where the object is written.
    */
    void selfDisplay ( std::ostream & out ) const;
    
    /**
       Checks the validity/consistency of the object.
       @return 'true' if the object is valid, 'false' otherwise.
    */
    bool isValid() const;

     

    // ------------------------------ internals ------------------------------
  private:

    Polynomial make( const detail::power_node & pnode )
    {
      return Xe_k<n, Ring>( pnode.k, pnode.e );
    }

    Polynomial make( const detail::monomial_node & mnode )
    {
      Polynomial m;
      if ( mnode.powers.size() != 0 )
        {
          m = make( mnode.powers[ 0 ] );
          for ( unsigned int i = 1; i < mnode.powers.size(); ++i )
            m *= make( mnode.powers[ i ] );
        }
      else
        m = 1;
      return ( (Ring) mnode.coef ) * m;
    }

    Polynomial make( const detail::polynomial_node & poly )
    {
      Polynomial p;
      for ( unsigned int i = 0; i < poly.monomials.size(); ++i )
        {
          p += make( poly.monomials[ i ] );
        }
      return p;
    }

    struct ExprNodeMaker : boost::static_visitor<> {
      Polynomial myP;
      MPolynomialReader & myPR;
      ExprNodeMaker( MPolynomialReader & reader )
        : myPR( reader )
      {}
      void operator()( const detail::polynomial_node & polynode)
      {
        myP = myPR.make( polynode );
      }
      void operator()( const detail::top_node & topnode)
      {
        myP = myPR.make( topnode );
      }
    };

    Polynomial make( const detail::top_node & topnode )
    {
      Polynomial p;
      if ( topnode.op ==  "T" )
        {
          // std::cerr << topnode.op << "-node";
          ExprNodeMaker emaker( *this );
          boost::apply_visitor( emaker, topnode.expressions[ 0 ] );
          p = emaker.myP;
        }
      else if ( topnode.op ==  "()" )
        {
          // std::cerr << topnode.op << "^" << topnode.exp <<"-node";
          ExprNodeMaker emaker( *this );
          boost::apply_visitor( emaker, topnode.expressions[ 0 ] );
          p = (Ring) 1; // emaker.myP;
          for ( unsigned int i = 1; i <= topnode.exp; ++i )
            p *= emaker.myP;
        }
      else if ( topnode.op == "*" )
        {
          // std::cerr << "*-node";
          ExprNodeMaker emaker( *this );
          boost::apply_visitor( emaker, topnode.expressions[ 0 ] );
          p = emaker.myP;
          boost::apply_visitor( emaker, topnode.expressions[ 1 ] );
          p *= emaker.myP;
        }
      else if ( topnode.op == "+" )
        {
          // std::cerr << "+-node";
          ExprNodeMaker emaker( *this );
          boost::apply_visitor( emaker, topnode.expressions[ 0 ] );
          p = emaker.myP;
          boost::apply_visitor( emaker, topnode.expressions[ 1 ] );
          p += emaker.myP;
        }
      else if ( topnode.op == "-" )
        {
          // std::cerr << "--node";
          ExprNodeMaker emaker( *this );
          boost::apply_visitor( emaker, topnode.expressions[ 0 ] );
          p = emaker.myP;
          boost::apply_visitor( emaker, topnode.expressions[ 1 ] );
          p -= emaker.myP;
        }
      else
        std::cerr << "[UNKNOWN-node]" << topnode.op << std::endl;
      return p;
    }
                 
    // ------------------------- Datas --------------------------------------
  private:
      
      
    // ------------------------- Hidden services ----------------------------
  protected:
    

  }; // end of class MPolynomialReader


  /**
   * Overloads 'operator<<' for displaying objects of class 'MPolynomialReader'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'MPolynomialReader' to write.
   * @return the output stream after the writing.
   */
  template <int n, typename TRing, typename TIterator>
  std::ostream&
  operator<< ( std::ostream & out,
               const MPolynomialReader<n, TRing, TIterator> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/math/MPolynomialReader.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MPolynomialReader_h

#undef MPolynomialReader_RECURSES
#endif // else defined(MPolynomialReader_RECURSES)
