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
 * @file ImageContainerByHashTree.h
 * @author Nicolas Silva (\c nicolas.silva@insa-lyon.fr )
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/09/02
 *
 * Header file for module ImageContainerByHashTree.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageContainerByHashTree_RECURSES)
#error Recursive header files inclusion detected in ImageContainerByHashTree.h
#else // defined(ImageContainerByHashTree_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageContainerByHashTree_RECURSES

#if !defined ImageContainerByHashTree_h
/** Prevents repeated inclusion of headers. */
#define ImageContainerByHashTree_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/images/CValue.h"
#include "DGtal/kernel/domains/CBoundedDomain.h"
#include "DGtal/base/Bits.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/images/Morton.h"
#include "DGtal/io/Color.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  namespace experimental
  {
    /////////////////////////////////////////////////////////////////////////////
    // template class ImageContainerByHashTree
    /**
     * Description of template class 'ImageContainerByHashTree' <p>
     * @brief Model of CImageContainer implementing the association
     * key<->Value using a hash tree. This class provides a built-in
     * iterator.
     *
     * @todo doc here
     *
     * The ImageContainerByHashTree relies on a tree model implemented
     * using a hash table based on Morton Codes.  A Morton hash key is
     * obatained from coordinates by interleaving the binary
     * representations of the coordinate values.  This means the
     * coordinates have to be of integer type for the morton code to be
     * valid.
     *
     * The data can be accessed using keys, coordinates or Iterators
     * provided by the container.
     *
     * The parent of a given key can be found by simply shifting to the
     * left the key's bits by it's dimension.  exemples: for an octree
     * (N = 3) the parent key of the key 1110100001 is 1110100.  for a
     * quadtree (N = 2) the parent key of the key 10010111 is 100101.
     *
     * It is then easy to determine, for a N-Dimmensionnal container,
     * all the children keys of a given key, by concatenating any
     * combination of N bits at the right side of the key (least
     * important bits).  exemple: for a quadtree (N = 2) 1010100,
     * 1010101, 1010110 and 1010111 are the children of 10101.
     *
     * In order to disgard any ambiguous case, we need to make the
     * assertion that the root of the hash is at key 1.  Else we would
     * have to deal with the problem of the key 00000..0 which would
     * have one of it's children equal to 0 and so on...  This means
     * that we need to set the key's N*maxDepth bit to 1 after
     * interleaving the corrdinates bit for key generation.
     *
     * Note that not any combination of bits is valid. We saw the
     * exemple of the key 0, and there are other cases of
     * invalidity. For a key to be valid, the last set bit (most
     * important set bit) must be at a position of the form k*N with N
     * the Dimmension of the Domain and k a positive integer.  exemples:
     * for an octree(N = 3) 1, 1000, 1010111 are valid keys, whereas 0,
     * 10, 11101 are not !
     *
     * Warning ! For performances this container's access method never
     * check for a key's validity.  Trying to access an invalid key may
     * destroy the validity off the tree's structure and/or get the
     * program stuck into an infinite loop.
     *
     * The method isKeyValid(..) is provided to verify the validity of a
     * key. Note that using this security strongly affects performances.
     *
     * @tparam TDomain type of domains
     * @tparam TValue type for image values
     * @taparam THashKey  type to store Morton keys
     * (default: DGtal::uint64_t)
     * 
     * @see testImageContainerByHashTree.cpp
     *       
     * */
    template < typename TDomain, typename TValue, typename THashKey = typename DGtal::uint64_t >
    class ImageContainerByHashTree
    {

    protected:
      class Node;
      template <int X, unsigned int exponent> struct POW;


    public:

      BOOST_CONCEPT_ASSERT(( CValue<TValue> ));
      BOOST_CONCEPT_ASSERT(( CBoundedDomain<TDomain> ));
				
      typedef THashKey HashKey;
      typedef TValue Value;
      typedef TDomain Domain;
      typedef typename Domain::Point Point;
      typedef typename Domain::Vector Vector;

      // static constants
      static const typename TDomain::Space::Dimension dim = TDomain::dimension;
      static const typename TDomain::Space::Dimension dimension = TDomain::dimension;
      static const unsigned int NbChildrenPerNode = POW<2, dimension>::VALUE;

      static const HashKey ROOT_KEY = static_cast<HashKey>(1);

      /**
       * The constructor.
       *
       * @param hashKeySize Number of bit of the hash key. This
       * parameter is important as it influences the amount of
       * collisions in the hash table. A value K creates an array of
       * length 2^K with potential unused cells so a compromise between
       * speed and memory usage is to be done here.
       *
       * @param depth Determines the maximum depth of the tree and thus
       * qthe "size" of the image. Each span then extends from 0 to
       * 2^depth.
       *
       * @param defaultValue In order for the tree to be valid it needs
       * a default value at the root (key = 1)
       */
      ImageContainerByHashTree(const unsigned int hashKeySize,
			       const unsigned int depth,
			       const Value defaultValue);

      ImageContainerByHashTree(const unsigned int hashKeySize,
			       const Point & p1,
			       const Point & p2,
			       const Value defaultValue);

      ImageContainerByHashTree(const ImageContainerByHashTree<Domain, Value>& toCopy);



      /**
       * Returns the value corresponding to a key.
       *
       * This is the generic method working with any valid key.  For
       * efficiency no check is performed on the key so be careful when
       * calling this method.  If the leafs are deeper than the
       * requested key, this method will recursively browse through the
       * children nodes and compute a average of each child's value
       * using blendChildren(..) which is very slow. Thus performances
       * are much better when accessing a leaf from a deeper key (needs
       * no blending).
       * @param key the haskkey
       * @return the value
       */
      Value get(const HashKey key) const;


      /**
       * Returns the value at a given key.
       *
       * @param key the hash key used as an index.
       * @return the value
       */
      Value operator () (const HashKey key) const;

      /**
       * Returns the value at a given point.
       *
       * @param aPoint The point
       * @return the value 
       */
      Value operator()(const Point &aPoint) const;

      /**
       * Returns the value at a given coordinate using upwardGet().
       * @param aPoint The point
       * @return the value
       */
      Value get(const Point & aPoint) const;


      /**
       * Returns the value corresponding to a key making the assomption
       * that the key is at same depth or deeper than the leaf we are
       * looking for.
       * @param key The key
       * @return the value
       */
      Value upwardGet(const HashKey key) const ;

      /**
       * A attempt to do the same thing as get(HashKey) but looking for
       * deeper leafs in the first place instead of doing this in the
       * second place. It hasn't show better results so far.
       * @param key The key. 
       * @return the value
       */
      Value reverseGet(const HashKey key) const;


      /**
       * Sets the value of an element in the container, creating it if
       * necessary. In order to keep the tree's coherence this method
       * may add and remove several other nodes in the tree so
       * performances strongly depend on wether or not and how much the
       * tree's structure needs to be modified.  For efficiency no check
       * is performed on the key
       * @param key The key
       * @param object The associated object 
       */
      void setValue(const HashKey key, const Value object);

      /**
       * Sets the value of an element in the container, creating it if
       * necessary. In order to keep the tree's coherence this method
       * may add and remove several other nodes in the tree so
       * performances strongly depend on wether or not and how much the
       * tree's structure needs to be modified.  For efficiency no check
       * is performed on the coordinates
       * @param key The point
       * @param object the associated object
       */
      void setValue(const Point& aPoint, const Value object);

      /**
       * Returns the size of a dimension (the container represents a
       * line, a square, a cube, etc. depending on the dimmension so no
       * need for distinctions between width, height, ect.)
       * @return the dimension size
       */
      inline unsigned int getSpanSize() const
      {
	return mySpanSize;
      }

      /**
       *  Returns the tree's depth.
       * @return the depth
       */
      inline unsigned int getDepth() const
      {
	return myTreeDepth;
      }

      /**
       *  Returns true if the key is valid.  A key is valid if the the
       * most important bit that is equal to 1 is at a position of the
       * type dim*k with dim the dimmension of the container (template
       * parameter) and k a strictly positive integer.
       * @param key the key
       * @return the boolean result
       */
      bool isKeyValid(HashKey key) const;

      /**
       * Checks recursively that the sub-tree starting with key is
       * valid.  A tree is valid if there's one (and only one) leaf for
       * each position at maximal depth.
       * @param key the key
       * @param leafAbove  
       */
      bool checkIntegrity(HashKey key = ROOT_KEY, bool leafAbove = false) const;

      int myDebugCounter; //debug

      //stuff that might be moved out of the class for reusability
      HashKey getKey(const Point & aPoint) const;

      unsigned int getKeyDepth(HashKey key) const;

      int* getCoordinatesFromKey(HashKey key) const;

      /**
       * Save the container into a stream.
       */
      void save(std::ostream& out);

      /**
       * Load the container from a stream.
       */
      void load(std::istream& in);

      /**
       * Prints in the state of the container as a tree. (Calls
       * printTree)
       */
      void printState(std::ostream& out, bool displayKeys = false) const;

      /**
       * Prints the sub-tree starting with the node corresponding to
       * key.
       */
      void printTree(HashKey key, std::ostream& out, bool displayKeys) const;

      /**
       * Prints the state of the container in a way that displays the
       * hash table and every node as it is stored in memory instead of
       * the usual tree representation.
       */
      void printInternalState(std::ostream& out, unsigned int nbBits = 0) const;

      /**
       * Prints informations about the state of the container without
       * displaying the data:
       *
       * 		- Nunber of elements in the tree.
       * 		- Amount of unused disk space due to blanks in the hash table.
       * 		- The dimmension.
       * 		- The number of bits of the HashKey.
       * 		- The size of the image.
       * 		- The average and the maximum amount of collisions.
       * 		- The total memory usage.
       */
      void printInfo(std::ostream& out) const;

      /**
       * Returns the number of empty places in the hash table.
       */
      unsigned int getNbEmptyLists() const;

      /**
       * Returns The average number of collisions in the hash table,
       * without counting the empty places.
       */
      double getAverageCollisions() const;

      /**
       * Returns the highest number of collisions in the hash table.
       */
      unsigned int getMaxCollisions() const;

      /**
       * Returns the number of elements for a givent key of the hash
       * table.
       */
      unsigned int getNbNodes(unsigned int intermediateKey) const;

      /**
       * Returns the total amount of elements in the container.
       */
      unsigned int getNbNodes()const;


      // -------------------------------------------------------------
      //  Iterator inner-class
      // -------------------------------------------------------------
      class Iterator
      {
      public:
	Iterator(Node** data, unsigned int position, unsigned int arraySize)
	{
	  myArraySize = arraySize;
	  myContainerData = data;
	  myNode = data[0];
	  myCurrentCell = position;
	  while ((!myNode) && (myCurrentCell < myArraySize))
	    {
	      myNode = myContainerData[++myCurrentCell];
	    }
	}
	bool isAtEnd()const
	{
	  return myCurrentCell >= myArraySize;
	}
	Value& operator*()
	{
	  return myNode->getObject();
	}
	bool operator ++ ()
	{
	  return next();
	}
	bool operator == (const Iterator& it)
	{
	  if (isAtEnd() && it.isAtEnd())
	    return true;
	  else
	    return (myNode == it.myNode);
	}
	bool operator != (const Iterator& it)
	{
	  if (isAtEnd() && it.isAtEnd())
	    return false;
	  else
	    return (myNode != it.myNode);
	}
	inline HashKey getKey() const
	{
	  return myNode->getKey();
	}
	bool next();
      protected:
	Node* myNode;
	unsigned int myCurrentCell;
	unsigned int myArraySize;
	Node** myContainerData;
      };

      /**
       * Returns an iterator to the first value as stored in the container.
       */
      Iterator begin()
      {
	return Iterator(myData, 0, myArraySize);
      }

      /**
       * Returns an iterator to the last value as stored in the container.
       */
      Iterator end()
      {
	return Iterator(myData, myArraySize, myArraySize);
      }

      void selfDisplay(std::ostream & out);

      bool isValid() const
      {
	return checkIntegrity();
      }

      // ------------- realization CDrawableWithBoard2D --------------------
    private:
      struct DefaultDrawStyle : public DrawableWithBoard2D
      {
	virtual void selfDraw(Board2D & aboard)
	{
	  aboard.setPenColorRGBi(60, 60, 60);
	  aboard.setLineStyle(Board2D::Shape::SolidStyle);
	}
      };

    public:
      /**
       * Default drawing style object.
       * @return the dyn. alloc. default style for this object.
       */
      DrawableWithBoard2D* defaultStyle() const;

      /**
       * @return the style name used for drawing this object.
       */
      std::string styleName() const;


      /**
       * Draw the object on a LibBoard board.
       * @param board the output board where the object is drawn.
       * @param minValue the minimum value contained in the image (used in the colormap settings)
       * @param maxValue the maximum value contained in the image (used in the colormap settings)
       * @tparam Coloramp any models of CColormap.
       */
      template<typename Colormap>
      void selfDraw(Board2D & board, const Value & minValue, const Value & maxValue ) const;
				
    protected:

      template <typename C>
      void
      recursiveDraw(HashKey key, const double p1[2], const double len, LibBoard::Board & board, const C& cmap) const;


      // -------------------------------------------------------------
      /**
       * @class ImageContainerByHashTree::Node
       *
       * An internal class that corresponds to a node of a linked list (as the hashTable points to linked
       * lists to handle collisions). Each element in the container is placed in a Node.
       */
      class Node
      {
      public:
	Node(Value object, HashKey key)
	{
	  myData = object;
	  myKey = key;
	}
	inline Node* getNext()
	{
	  return myNext;
	}
	inline void setNext(Node* next)
	{
	  myNext = next;
	}
	inline HashKey getKey()
	{
	  return myKey;
	}
	inline Value& getObject()
	{
	  return myData;
	}
	~Node() { }
      protected:
	HashKey myKey;
	Node* myNext;
	Value myData;
      };// -----------------------------------------------------------


        /**
         * Template metaprogrammed power function.  To compute at
         * compilation time the number of children per node (N) and save
         * runtime calculations
         */
      template <int X, unsigned int exponent> struct POW
      {
	enum { VALUE = X * POW < X, exponent - 1 >::VALUE};
      };
      template <int X > struct POW<X, 1>
      {
	enum { VALUE = X };
      };

      /**
       * This is part of the hash function. It is called whenever a key
       * is accessed.  The mask used to compute the result is
       * precomputed in the constructor for efficiency.
       */
      inline HashKey getIntermediateKey(const HashKey key) const;


      /**
       * Add a Node to the tree.  This method is very used when writing
       * in the tree (set method).
       */
      Node* addNode(const Value object, const HashKey key)
      {
	Node* n = getNode(key);
	if (n)
          {
            n->getObject() = object;
            //n->setObject(object);
            return n;
          }
	n = new Node(object, key);
	HashKey key2 = getIntermediateKey(key);
	n->setNext(myData[key2]);
	myData[key2] = n;
	return n;
      }

      /**
       * Returns a pointer to the node corresponding to the key. If it
       * does'nt exist, returns 0.  This method is called VERY often,
       * and thus should operate as fast as possible.
       * @param key The key.
       * @return the pointer to the node corresponding to the key.
       */
      inline Node* getNode(const HashKey key)	const	// very used !!
      {
	Node* iter = myData[getIntermediateKey(key)];
	while (iter != 0)
          {
            if (iter->getKey() == key)
              return iter;
            iter = iter->getNext();
          }
	return 0;
      }

      /**
       * Remove the node corresponding to a key. Returns false if the
       * node doesn't exist.
       * @param key The key
       */
      bool removeNode(HashKey key);

      /**
       * Recusrively calls RemoveNode on the key and its children.
       * @param key The key. 
       * @param nbRecursions the number of recursions performed.
       */
      void recursiveRemoveNode(HashKey key, unsigned int nbRecursions);


      /**
       * Set the (maximum) depth of the tree and precompute a mask used
       * for some calculations.  The depth of the tree must be known
       * because accessing the data from coordinates depends on it.
       * @param depth the maxumum depth.
       */
      void setDepth(unsigned int depth);

      /**
       * Recursively get the value of all the leafs below and blend it
       * to get the value of a parent node. This is called in the get()
       * method when it has been determined that the leafs are deeper
       * than the requested key.
       */
      Value blendChildren(HashKey key) const;

      /**
       * The array of linked lists containing all the data
       */
      Node** myData;

      /**
       * The size of the intermediate hashkey. The bigger the less
       * collisions, but at the same time the more chances to have
       * unused memory allocated.
       */
      unsigned int myKeySize;

      unsigned int myArraySize;

      /**
       * The depth of the tree
       */
      unsigned int myTreeDepth;

      unsigned int mySpanSize;

      Point myOrigin;

      /**
       * Precoputed masks to avoid recalculating it all the time
       */
      HashKey myDepthMask;
      HashKey myPreComputedIntermediateMask; // ~((~0) << _keySize)

      ///The morton code computer.
      Morton<HashKey, Point> myMorton;

    };

    /**
     * Overloads 'operator<<' for displaying objects of class 'ImageContainerByHashTree'.
     * @param out the output stream where the object is written.
     * @param object the object of class 'ImageContainerByHashTree' to write.
     * @return the output stream after the writing.
     */
    template<typename TDomain, typename TValue, typename THashKey >
    std::ostream&
    operator<< ( std::ostream & out,  ImageContainerByHashTree<TDomain, TValue, THashKey> & object )
    {
      object.selfDisplay( out);
      return out;
    }

  }

} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/ImageContainerByHashTree.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageContainerByHashTree_h

#undef ImageContainerByHashTree_RECURSES
#endif // else defined(ImageContainerByHashTree_RECURSES)
