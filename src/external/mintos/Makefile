###################################################################
# Configure these settings to locate your GLPK include director and
# lib directory
###################################################################
GLPK = /usr/include/glpk
GLPK_LIB = /usr/lib

#optional: set compiling flags here
CC = g++
AR = ar
CPPFLAGS = -O2 -Wall

INCDIRS = include $(GLPK)
# may have to edit these depending on whether you have isinf, isnan, and finite
# on your system
DEFINES = HAVE_DECL_ISINF HAVE_DECL_ISNAN HAVE_DECL_FINITE HAVE_GLPK=1

FLAGS = $(CPPFLAGS) $(addprefix -I, $(INCDIRS)) $(addprefix -D, $(DEFINES))

SRCS = $(wildcard src/*.cpp)
OBJS= $(SRCS:.cpp=.o)

###########################################################################
.PHONY: default lib clean docs

default: lib test timeopt

test: lib
	$(CC) $(FLAGS) test.cpp Timer.cpp -Llib -L$(GLPK_LIB) -lmintos -lglpk -o $@

contacttest: lib
	$(CC) $(FLAGS) contacttest.cpp Timer.cpp -Llib -L$(GLPK_LIB) -lmintos -lglpk -o $@

timeopt: lib
	$(CC) $(FLAGS) timeopt.cpp Timer.cpp -Llib -L$(GLPK_LIB) -lmintos -lglpk -o $@

docs:
	doxygen doxygen.conf

clean:
	rm -rf src/*.o lib/libmintos.a docs test trajopt

%.o: %.cpp
	$(CC) $(FLAGS) -c $*.cpp -o $*.o

lib: $(OBJS)
	$(AR) rcs lib/libmintos.a src/*.o
