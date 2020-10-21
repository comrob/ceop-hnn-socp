#
# Date:      2011/07/11 17:55
# Author:    Jan Faigl
#

CXX:=ccache $(CXX)

include Mk/libs.mk

CXX+=$(dynamic:yes=LD_LIBRARY_PATH=$(CPLEXBINDIR))

CPPFLAGS+=$(LOCAL_CFLAGS) 
LDFLAGS+=$(LOCAL_LDFLAGS)

CPPFLAGS+=$(CRL_CFLAGS) $(BOOST_CFLAGS) $(CAIRO_CFLAGS) $(LOG4CXX_CPPFLAGS) $(CPLEX_CFLAGSI)
LDFLAGS+=$(CRL-ALGORITHM) $(CRL-GUI_LDFLAGS) $(CRL_LDFLAGS) $(CAIRO_LDFLAGS) $(BOOST_LDFLAGS) $(LOG4CXX_LDFLAGS) $(CPLEX_CFLAGSI) $(CPLEX_LDFLAGS)

CXXFLAGS+=-std=c++11
CXXFLAGS+=-g
CXXFLAGS+=-O3 -DEIGEN_NO_DEBUG
CXXFLAGS+=$(CPLEX_CFLAGS)

OBJS=\
     src/hnn.o\
     src/coords.o\
     src/hnn_graph.o\
     src/hnn_nn.o\
     src/hnn_route.o\
     src/thnn_ceop.o

TARGET=thnn_ceop

include Mk/comrob.mk
