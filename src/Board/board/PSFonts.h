/* -*- mode: c++ -*- */
/**
 * @file   PSFonts.h
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Sat Aug 18 2007
 * 
 * @brief  The Point structure.
 * @copyright
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 */
#ifndef _BOARD_PSFONTS_H_
#define _BOARD_PSFONTS_H_

#include <cmath>

namespace LibBoard {

/*
 * Postscript Type 1 base fonts :
 * ------------------------------
 * ITC Avant Garde Gothic (Book, Book Oblique, Demi, Demi Oblique)
 * ITC Bookman (Light, Light Italic, Demi, Demi Italic)
 * Courier (Regular, Oblique, Bold, Bold Oblique)
 * Helvetica (Regular, Oblique, Bold, Bold Oblique, Condensed, Condensed Oblique, Condensed Bold, Condensed Bold Oblique)
 * New Century Schoolbook (Roman, Italic, Bold, Bold Italic)
 * Palatino (Roman, Italic, Bold, Bold Italic)
 * Symbol
 * Times (Roman, Italic, Bold, Bold Italic)
 * ITC Zapf Chancery (Medium Italic)
 * ITC Zapf Dingbats
 */
  namespace Fonts {
    enum Font {
      TimesRoman,
      TimesItalic,
      TimesBold,
      TimesBoldItalic,
      AvantGardeBook,
      AvantGardeBookOblique,
      AvantGardeDemi,
      AvantGardeDemiOblique,
      BookmanLight,
      BookmanLightItalic,
      BookmanDemi,
      BookmanDemiItalic,
      Courier,
      CourierOblique,
      CourierBold,
      CourierBoldOblique,
      Helvetica,
      HelveticaOblique,
      HelveticaBold,
      HelveticaBoldOblique,
      HelveticaNarrow,
      HelveticaNarrowOblique,
      HelveticaNarrowBold,
      HelveticaNarrowBoldOblique,
      NewCenturySchoolbookRoman,
      NewCenturySchoolbookItalic,
      NewCenturySchoolbookBold,
      NewCenturySchoolbookBoldItalic,
      PalatinoRoman,
      PalatinoItalic,
      PalatinoBold,
      PalatinoBoldItalic,
      Symbol,
      ZapfChanceryMediumItalic,
      ZapfDingbats
    };
  } // namespace Fonts

  extern const char * PSFontNames[];

} // mamespace LibBoard

#endif // _BOARD_PSFONTS_H_

