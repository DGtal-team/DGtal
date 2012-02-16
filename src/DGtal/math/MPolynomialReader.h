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
  struct power_node {
    int k;
    int e;
  };
  struct monomial_node {
    double coef;
    std::vector<power_node> powers;
  };
  struct polynomial_node {
    std::vector<monomial_node> monomials;
  };

  struct top_node;
  typedef 
  boost::variant< boost::recursive_wrapper<top_node>, polynomial_node  >
  expr_node;

  struct top_node {
    std::string op;
    std::vector<expr_node> expressions;
    int exp;
  };


}

BOOST_FUSION_ADAPT_STRUCT(
                          DGtal::power_node,
                          (int, k)
                          (int, e)
)
BOOST_FUSION_ADAPT_STRUCT(
                          DGtal::monomial_node,
                          (double, coef)
                          (std::vector<DGtal::power_node>, powers)
)
BOOST_FUSION_ADAPT_STRUCT(
                          DGtal::polynomial_node,
                          (std::vector<DGtal::monomial_node>, monomials)
)
BOOST_FUSION_ADAPT_STRUCT(
    DGtal::top_node,
    (std::string, op)
    (std::vector<DGtal::expr_node>, expressions)
    (int, exp)
)

namespace DGtal
{
  namespace qi = boost::spirit::qi;
  namespace ascii = boost::spirit::ascii;
  namespace phoenix = boost::phoenix;

  template <typename Iterator>
  struct MPolynomialGrammar 
    : qi::grammar<Iterator, top_node(), ascii::space_type>
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
      //using boost::phoenix::ref;
      using phoenix::at_c;
      using phoenix::push_back;

      top = 
        mulexpr [push_back(at_c<1>(_val), _1)] 
        >> ( ( lit('+') [ at_c<0>(_val) = "+" ] 
               >> top [push_back(at_c<1>(_val), _1)]  )
             | eps [ at_c<0>(_val) = "T" ] );

      mulexpr =
        subexpr [push_back(at_c<1>(_val), _1)] 
        >> ( ( lit('*') [ at_c<0>(_val) = "*" ] 
               >> mulexpr [push_back(at_c<1>(_val), _1)] )
             | eps [ at_c<0>(_val) = "T" ] );

      subexpr = 
        mpolynomial
        | expexpr;
      expexpr = ( lit('(') [ at_c<0>(_val) = "()" ] 
                  >> top [ push_back(at_c<1>(_val), _1) ]
                  >> lit(')') >> ( ( lit('^') >> int_ [ at_c<2>(_val) = _1 ] )
                                   | eps [ at_c<2>(_val) = 1 ]  ) );
        
      mpolynomial =
        monomial [push_back(at_c<0>(_val), _1)]
        >> *( lit('+') >> monomial [push_back(at_c<0>(_val), _1)] );
      monomial = double_ [at_c<0>(_val) = _1]
        >> *( variable [push_back(at_c<1>(_val), _1)] );
      //>> *( variable [push_back(at_c<1>(_val), _1)] );
      variable = 
        lit('X') >> lit('_') 
                 >> int_ [at_c<0>(_val) = _1]
                 >> ( ( lit('^') >> int_ [at_c<1>(_val) = _1] ) // X_k^e
                      | eps [at_c<1>(_val) = 1] // X_k
                      );
    }
    qi::rule<Iterator, top_node(), ascii::space_type> top;
    qi::rule<Iterator, top_node(), ascii::space_type> mulexpr;
    qi::rule<Iterator, expr_node(), ascii::space_type> subexpr;
    qi::rule<Iterator, top_node(), ascii::space_type> expexpr;
    qi::rule<Iterator, polynomial_node(), ascii::space_type> mpolynomial;
    qi::rule<Iterator, monomial_node(), ascii::space_type> monomial;
    qi::rule<Iterator, power_node(), ascii::space_type> variable;

  };
}

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class MPolynomialReader
  /**
     Description of template class 'MPolynomialReader' <p> \brief Aim: 
  */
  template <int n, typename TRing>
  class MPolynomialReader
  {
  public:
    typedef TRing Ring;
    typedef MPolynomial<n, Ring, std::allocator<Ring> > Polynomial;
    
    Polynomial make( const power_node & pnode )
    {
      //std::cerr << "X_" << pnode.k << "^" << pnode.e << " ";
      return Xe_k<n, Ring>( pnode.k, pnode.e );
    }
    Polynomial make( const monomial_node & mnode )
    {
      Polynomial m;
      // std::cerr << mnode.coef << "*";
      if ( mnode.powers.size() != 0 )
        {
          m = make( mnode.powers[ 0 ] );
          for ( unsigned int i = 1; i < mnode.powers.size(); ++i )
            m *= make( mnode.powers[ i ] );
        }
      else
        {
          power_node pnode = { 0, 0 };
          m = 1; // make( pnode );
        }
      return mnode.coef * m;
    }
    Polynomial make( const polynomial_node & poly )
    {
      Polynomial p;
      for ( unsigned int i = 0; i < poly.monomials.size(); ++i )
        {
          // if ( i != 0 ) std::cerr << "+";
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
      void operator()( const polynomial_node & polynode)
      {
        myP = myPR.make( polynode );
      }
      void operator()( const top_node & topnode)
      {
        myP = myPR.make( topnode );
      }
    };

    Polynomial make( const top_node & topnode )
    {
      Polynomial p;
      if ( topnode.op ==  "T" )
        {
          std::cerr << topnode.op << "-node";
          ExprNodeMaker emaker( *this );
          boost::apply_visitor( emaker, topnode.expressions[ 0 ] );
          p = emaker.myP;
        }
      else if ( topnode.op ==  "()" )
        {
          std::cerr << topnode.op << "^" << topnode.exp <<"-node";
          ExprNodeMaker emaker( *this );
          boost::apply_visitor( emaker, topnode.expressions[ 0 ] );
          p = (double) 1; // emaker.myP;
          for ( unsigned int i = 1; i <= topnode.exp; ++i )
            p *= emaker.myP;
        }
      else if ( topnode.op == "*" )
        {
          std::cerr << "*-node";
          ExprNodeMaker emaker( *this );
          boost::apply_visitor( emaker, topnode.expressions[ 0 ] );
          p = emaker.myP;
          boost::apply_visitor( emaker, topnode.expressions[ 1 ] );
          p *= emaker.myP;
        }
      else if ( topnode.op == "+" )
        {
          std::cerr << "+-node";
          ExprNodeMaker emaker( *this );
          boost::apply_visitor( emaker, topnode.expressions[ 0 ] );
          p = emaker.myP;
          boost::apply_visitor( emaker, topnode.expressions[ 1 ] );
          p += emaker.myP;
        }
      else
        std::cerr << "U-node" << topnode.op << std::endl;
      return p;
    }
                 

    MPolynomialReader() {}

    bool addMPolynomial( Polynomial & p, const string & expr )
    {
      using qi::phrase_parse;
      using ascii::space;
      std::string::const_iterator iter = expr.begin();
      std::string::const_iterator end = expr.end();
      typedef MPolynomialGrammar<std::string::const_iterator> MyGrammar;
      MyGrammar gpolynomial;
      top_node m;
      bool r = phrase_parse( iter, end, gpolynomial, space, m );
      if (r) p = make( m );
      if (iter != end) // fail if we did not get a full match
        return false;
      return r;
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
  template <int n, typename TRing>
  std::ostream&
  operator<< ( std::ostream & out,
               const MPolynomialReader<n, TRing> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/math/MPolynomialReader.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MPolynomialReader_h

#undef MPolynomialReader_RECURSES
#endif // else defined(MPolynomialReader_RECURSES)
