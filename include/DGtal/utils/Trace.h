/** 
 * @file Trace.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2009/12/19
 * 
 * Header file for module Trace.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Trace_RECURSES)
#error Recursive header files inclusion detected in Trace.h
#else // defined(Trace_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Trace_RECURSES

#if !defined Trace_h
/** Prevents repeated inclusion of headers. */
#define Trace_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <ostream>
#include <string>
#include <stack>
#include "DGtal/base/Common.h"
#include "DGtal/utils/Clock.h"
#include "DGtal/utils/TraceWriter.h"
#include "DGtal/utils/TraceWriterTerm.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  
  /////////////////////////////////////////////////////////////////////////////
  // class Trace
  /** 
   * Description of class 'Trace' <p>
   * Aim: implementation of basic methods to trace out messages with indentation levels.
   * 
   * Trace objects use a TraceWriter to switch between terminal and file outputs.
   * Methods postfixed with "Debug" contain no code if the compilation flag DEBUG is not set.
   *
   *
   * For usage examples, see the test_trace.cpp file.
   *
   * \see test_trace.cpp
   *
   * \todo Create a default constructor on TraceWriterTerm(std::cerr)
   */
  class Trace
  {
    // ----------------------- Standard services ------------------------------
  public:

    /// Pattern to be used to indent the messages.
#define TRACE_PATTERN "  "


    /**
     * Constructor.
     *
     * @param writer  the output writer that will receive the traces.
     * 
     */
    Trace(TraceWriter & writer);


    /**
     * Destructor. 
     */
    ~Trace();

   /**
    * Reset all the variables of the Trace object (indentation level and keyword stack)
    *
    */
   void reset();



    /**
     * Enter a new block and increase the indentation level
     * @param keyword contains a label to the new block
     *
     */
    void beginBlock(const std::string &keyword = "");

    
    /**
     * Debug version of Trace::beginBlock()
     * @param keyword contains a label to the new block
     *
     */
    void beginBlockDebug(const std::string &keyword = "");
 

    /**
     * Leave a current block, decrease the indentation level and display the associate keyword
     *
     * @return  the ellapsed time in the block in milliseconds (Class Clock).
     */
    long endBlock();
    

    /**
     * Debug version of Trace::endBlock()
     * @return  the ellapsed time in the block in milliseconds (Class Clock).
     *
     */
    long endBlockDebug();
 

    /**
     * Create a string with an indentation prefix for a normal trace.
     * @return the cerr output stream with the prefix
     */
    std::ostream & info() const;

    /**
     * Debug version of the Trace::info()
     * @return the cerr output stream with the prefix
     */
    std::ostream &infoDebug() const;


    /**
     * Create a string with an indentation prefix for a warning trace.
     *  the string is postfixed by the keyword "[WRNG]"
     * @return the cerr output stream with the prefix
     */
     std::ostream & warning() const;

     /**
     * Debug version of  Trace::warning()
     * @return the output stream with the prefix
     */
     std::ostream & warningDebug() const;


    /**
     * Create a string with an indentation prefix for an error trace.
     *  the string is postfixed by the keyword "[ERR]"
     * @return the cerr output stream with the prefix
     */
    std::ostream &  error() const;

    /**
     * Debug version of  Trace::error()
     * @return the output stream with the prefix
     */
    std::ostream & errorDebug() const;
   
    /**
     * Create a string with an indentation prefix for an emphased trace.
     *
     * @return the cerr output stream with the prefix
     */
    std::ostream &  emphase() const;

    /**
     * Debug version of the Trace::emphase()
     * @return the output stream with the prefix
     */
    std::ostream &emphaseDebug() const;

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:
    ///The indentation level
    unsigned int myCurrentLevel;
    ///The indentation prefix string
    std::string myCurrentPrefix;

    ///A stack to store the block keywords
    std::stack<std::string> myKeywordStack;

    ///A reference to the output writer
    TraceWriter &myWriter;

    ///A stack to store the block clocks
    std::stack<Clock*> myClockStack;

    // ------------------------- Hidden services ------------------------------
  protected:

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    INLINE Trace( const Trace & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    INLINE Trace & operator=( const Trace & other );



  
    // ------------------------- Internals ------------------------------------
  private:
  
  }; // end of class Trace


  /**
   * Overloads 'operator<<' for displaying objects of class 'Trace'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Trace' to write.
   * @return the output stream after the writing.
   */
  INLINE std::ostream&
  operator<<( std::ostream & out, const Trace & object );

  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/utils/Trace.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Trace_h

#undef Trace_RECURSES
#endif // else defined(Trace_RECURSES)
