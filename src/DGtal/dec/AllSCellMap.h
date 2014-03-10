#if !defined(__DEC_ALLSCELLMAP_H__)
#define __DEC_ALLSCELLMAP_H__

#include <map>
#include <string>

template <typename Calculus, typename V>
struct AllSCellMap : public std::map<typename Calculus::SCell, V>
{
		typedef typename Calculus::SCell SCell;
		typedef SCell Key;
		typedef V Value;

		typedef typename std::map<Key, Value> Container;
		typedef typename Container::const_iterator ConstIterator;
		typedef typename Container::iterator Iterator;

		AllSCellMap(const Calculus& _calculus);

		void writeImage(const std::string& filename, const Value& value_outside = -1, const Value& value_inside_default = 0) const;

		template <typename Viewer, typename ColorMap>
		void display3D(Viewer& viewer, const ColorMap& color_map) const;

		template <typename Board, typename ColorMap>
		void display2D(Board& board, const ColorMap& color_map) const;

		const Calculus& calculus;
};

#include "AllSCellMap.ih"

#endif

