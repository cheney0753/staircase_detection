

MARCH		:= #i686
MEX_SUFFIX	:= #mexglx

MEX		:= /usr/local/MATLAB/R2013b/bin/mex
#/usr/local/MATLAB/R2012b/bin/mex

#for opencv in common location
OPENCV_PC       := opencv
#opencv in a custom location, use opencv.pc file
#OPENCV_PC       := /usr/local/stow/OpenCV-2.4.6.1/lib/pkgconfig/opencv.pc

#LAPACK_LOCATION := /opt/matlabR2009a/bin/glnx86/
#LAPACK_LOCATION := /usr/local/matlab-2007b/bin/glnx86/

MEXCFLAGS	:= -Wall -O3 --fast-math -fPIC  -ftree-vectorize  -funroll-loops -I  #-march=$(MARCH) -mtune=$(MARCH)
MEXCFLAGS       += $(shell pkg-config --cflags $(OPENCV_PC))
#MEX_CXXFLAGS 	:= CFLAGS='$(MEXCFLAGS)' 
MEX_CXXFLAGS 	:= CXXFLAGS='$(MEXCFLAGS)' 

#print output
#MEXCFLAGS       +=-DVERBOSE
#MEXCFLAGS       +=-DVERBOSE_RANSAC

LDFLAGS		:= -lm
LDFLAGS         += $(shell pkg-config --libs $(OPENCV_PC) )
MEX_CFLAGS 	:= CFLAGS='$(MEXCFLAGS)' 

CFLAGS          += $(shell pkg-config --cflags $(OPENCV_PC) )
CXXFLAGS        += $(shell pkg-config --cflags $(OPENCV_PC) ) 
MEX_CXX         := g++-4.4
CXX             := g++-4.4
CC		:= gcc-4.4

#LAPACK
MEX_CFLAGS      += -ILAPACK/ 
CFLAGS          += -ILAPACK/  -lblas -llapack
CXXFLAGS        += -ILAPACK/  -lblas -llapack 

#openmp
#in Makefile

#LAPACK_LOCATION := /opt/matlabR2009a/bin/glnx86/
MEX_LDFLAGS     = $(LDFLAGS) -lmwlapack  
#$(LAPACK_LOCATION)/libmwblas.so

