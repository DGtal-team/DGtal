TEXSRC=cfd-abstract.tex
PS=${TEXSRC:.tex=.ps}

all: ${PS}

%.dvi: %.tex
	latex $<

%.ps: %.dvi
	dvips $< -o $@

clean:
	rm -f *~ ${PS} ${TEXSRC:.tex=.dvi} *.log *.out *.nav *.aux

