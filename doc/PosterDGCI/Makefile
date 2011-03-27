LATEX  = latex
BIBTEX = posterDGCI 
SRC    = posterDGCI
DVI    = dvips 
BIB    = bibtex
PDF_FILE   = $(SRC).pdf	
PS2PDF_OPTIONS = -dPDFSETTINGS=/prepress -dUseFlateCompression=true -sColorImageFilter=FlateEncode -sGrayImageFilter=FlateEncode -dAutoFilterColorImages=false  -dAutoFilterGrayImages=false -dEncodeColorImages=false  -dEncodeGrayImages=false  -dEncodeMonoImages=false
PS2PDF_OPTIONS2 = -dPDFSETTINGS=/prepress 
#texclean


all: clean
	$(LATEX) $(SRC) 
	$(BIB) $(BIBTEX) &
	($(LATEX) $(SRC) && $(LATEX) $(SRC)) 
	$(DVI) $(SRC).dvi -o

slidesFastTrack.pdf: slidesFastTrack.tex 
	$(LATEX) slidesFastTrack
	$(DVI) -Ppdf   slidesFastTrack.dvi -o
	ps2pdf $(PS2PDF_OPTIONS2)   slidesFastTrack.ps


slidesFastTrack.ps: slidesFastTrack.tex 
	$(LATEX) slidesFastTrack
	$(DVI) -Ppdf   slidesFastTrack.dvi -o


pdf: clean
	$(LATEX) $(SRC) 
	$(BIB) $(BIBTEX) &
	($(LATEX) $(SRC) && $(LATEX) $(SRC)) 
	$(DVI) -Ppdf   $(SRC).dvi -o
	ps2pdf $(PS2PDF_OPTIONS) $(SRC).ps

pdf2: clean
	$(LATEX) $(SRC) 
	$(BIB) $(BIBTEX) &
	($(LATEX) $(SRC) && $(LATEX) $(SRC)) 
	$(DVI) -Ppdf  -t landscape -t a0  $(SRC).dvi -o
	ps2pdf $(PS2PDF_OPTIONS2)  $(SRC).ps



clean:
	echo "Cleaning temp files"
	rm -f  *.toc *.aux *.cp *.fn *.tp *.vr *.pg *.ky *.bbl *.blg *.tex~ *.bib~

