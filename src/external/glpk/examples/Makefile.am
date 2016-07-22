## Process this file with automake to produce Makefile.in ##

INCLUDES = -I$(srcdir)/../include

LDADD = ../src/libglpk.la

bin_PROGRAMS = glpsol

glpsol_SOURCES = glpsol.c

check: glpsol$(EXEEXT)
	./glpsol$(EXEEXT) --version
	./glpsol$(EXEEXT) --mps $(srcdir)/plan.mps

## eof ##
