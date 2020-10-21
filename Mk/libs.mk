#
# Date:      2011/07/11 17:55
# Author:    Jan Faigl
#

OPSYS=$(shell uname)

PLATFORM=$(shell uname -p)
ARCH=.$(PLATFORM)

ifeq ($(OPSYS),FreeBSD)
   BOOST_CFLAGS=-I/usr/local/include
   BOOST_LDFLAGS=-L/usr/local/lib

else
   LOG4CXX_CPPFLAGS=$(shell pkg-config --cflags liblog4cxx)
   LOG4CXX_LDFLAGS=$(shell pkg-config --libs liblog4cxx)

   CAIRO_LDFLAGS:=-L/usr/X11/lib
   CAIRO_CFLAGS:=-I/usr/X11/include
endif


BOOST_LDFLAGS+=-lboost_program_options -lboost_thread -lboost_filesystem -lboost_iostreams -lboost_system
LOG4CXX_LDFLAGS+=-llog4cxx
CAIRO_LDFLAGS+=-lcairo -pthread -lX11

LOCAL_CFLAGS=-I./include
LOCAL_LDFLAGS=-L./lib

CRL-GUI_LDFLAGS=-lcrl-gui
CRL_LDFLAGS=-lcrl
CRL-ALGORITHM=-lcrl-algorithm


CAIRO_LDFLAGS+=-lcairo -pthread -lX11

# CPLEX flags
DIR        = $(shell locate "cplex/bin" | head -n 1 | sed -e "s/bin//" | sed -e "s/\/cplex//")
SYSTEM     = x86-64_linux
LIBFORMAT  = static_pic

CPLEXDIR      = $(shell echo ${DIR}cplex)
CONCERTDIR    = $(shell echo ${DIR}concert)

CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD
CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CPLEXLIB      = cplex$(dynamic:yes=1290)
CPLEXBINDIR   = $(CPLEXDIR)/bin/$(SYSTEM)

CCLNDIRS  = -L$(CPLEXLIBDIR) -L$(CONCERTLIBDIR) $(dynamic:yes=-L$(CPLEXBINDIR))
CCLNFLAGS = -lconcert -lilocplex -l$(CPLEXLIB) -lm -lpthread -ldl

CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include

CPLEX_CFLAGS=$(CCOPT)
CPLEX_CFLAGSI = -I$(CPLEXINCDIR) -I$(CONCERTINCDIR) $(CCLNDIRS)
CPLEX_LDFLAGS = $(CCLNFLAGS)
