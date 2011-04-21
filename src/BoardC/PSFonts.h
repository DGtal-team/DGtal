/* -*- mode: c++ -*- */
/**
 * @file   PSFonts.h
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#ifndef _BOARDCAIRO_PSFONTS_H_
#define _BOARDCAIRO_PSFONTS_H_

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

} // namespace

#endif

