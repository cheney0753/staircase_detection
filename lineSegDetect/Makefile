#author:	Jean-Philippe Tardif

MAKEFILEDEP     := Makefile ../Makefile.spec
ALLDEP 	        += $(MAKEFILEDEP)

#../CMINPACK/libminpack.a
include ../Makefile.spec


strDiv		:= "======================================================================="
mexfile 	:= mxEdgelink mxGetEdges
exefile 	:= mxGetEdges

all: $(mexfile) $(exefile)


mxEdgelink: mxEdgelink.cpp $(ALLDEP)
	@echo $(strDiv)
	@echo "   MEX '$<' ==> '$@'"	
	@echo ""
	@echo $(MEX) CXX=$(MEX_CXX) $(MEX_CXXFLAGS) $(CXXFLAGS) $(MEX_LDFLAGS) -DMATLAB 
	$(MEX) CXX=$(MEX_CXX) $(MEX_CXXFLAGS) $(CXXFLAGS) $(MEX_LDFLAGS) -DMATLAB   $< -o $@

mxGetEdges: mxGetEdges.cpp $(ALLDEP)
	@echo $(strDiv)
	@echo "   MEX '$<' ==> '$@'"	
	@echo ""
	#@echo $(MEX) CXX=$(MEX_CXX) $(MEX_CXXFLAGS) $(CXXFLAGS) $(MEX_LDFLAGS) -DMATLAB 
	$(MEX) -v CXX=$(MEX_CXX) $(MEX_CXXFLAGS) $(CXXFLAGS) $(MEX_LDFLAGS) -DMATLAB    $< -o $@ 


mxGetEdges.bin: mxGetEdges.cpp $(ALLDEP)
	@echo $(strDiv)
	@echo "   MEX '$<' ==> '$@'"	
	@echo ""
	@echo $(LDFLAGS)
	$(CXX)  $(CXXFLAGS) $(LDFLAGS)     $< -o $@  


clean:
	rm -f *.o 

