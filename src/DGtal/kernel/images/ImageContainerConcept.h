#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/concept/requires.hpp>

namespace DGtal
{
template <class TPoint, typename TValue, class TContainer>
struct ImageContainerConcept  : boost::Assignable<TValue>, boost::EqualityComparable<TValue>
{

 public:
    typedef typename TContainer::Iterator Iterator;

    BOOST_CONCEPT_ASSERT((boost::BidirectionalIterator<Iterator>));
    
    BOOST_CONCEPT_USAGE(ImageContainerConcept)
    {
        TContainer j(a,b);
        it=i.begin();  // require postincrement-dereference returning value_type	
        same_type(i(it),v);        // require preincrement returning X&
	same_type(i(a),v);
    }

 private:
    TContainer i;
    Iterator it;
    TValue v;
    TPoint a,b;

    // Type deduction will fail unless the arguments have the same type.
    template <typename T>
    void same_type(T const&, T const&);
};
}