#########################################################################
#                                VARIABLES                              #
#########################################################################


# Compilation flags and variables
CC = g++
# CFLAGS = -std=c++11 -O3 -DATOM_STATES -DNDEBUG -pthread
CFLAGS = -std=c++11 -g -DATOM_STATES -pthread

# Variables for directories
ID = include
SD = src
TD = test
OD = obj
ID_UTIL = $(ID)/util
SD_UTIL = $(SD)/util
ID_SOLV = $(ID)/solvers
ID_PPDDL = $(ID)/ppddl
SD_SOLV = $(SD)/solvers
OD_SOLV = $(OD)/solvers
OD_PPDDL = $(OD)/ppddl
ID_REDUCED = $(ID)/reduced
SD_REDUCED = $(SD)/reduced
OD_REDUCED = $(OD)/reduced
OD_SOLV_MOBJ = $(OD)/solvers/mobj
ID_SOLV_MOBJ = $(ID)/solvers/mobj
SD_SOLV_MOBJ = $(SD)/solvers/mobj

ID_DOMAINS = $(ID)/domains
SD_DOMAINS = $(SD)/domains
OD_DOMAINS = $(OD)/domains
# ID_MOBJ_DOMAINS = $(ID)/mobj/domains
# SD_MOBJ_DOMAINS = $(SD)/mobj/domains
# OD_MOBJ_DOMAINS = $(OD)/domains/mobj

SD_RS = $(SD_DOMAINS)/rocksample
ID_RS = $(ID_DOMAINS)/rocksample
SD_SR = $(SD_DOMAINS)/SearchRescue
ID_SR = $(ID_DOMAINS)/SearchRescue


# Variables for include directives
INCLUDE_DOM = -I$(ID_DOMAINS) 
INCLUDE_CORE = -I$(ID_UTIL) -I$(ID)
INCLUDE_SOLVERS = -I$(ID_SOLV) 
INCLUDE = $(INCLUDE_DOM) $(INCLUDE_CORE) $(INCLUDE_SOLVERS)

# Variables for source/header files
I_H = $(ID)/*.h
S_CPP = $(SD)/*.cpp
SOLV_CPP = $(SD_SOLV)/*.cpp
SOLV_H = $(ID_SOLV)/*.h
UTIL_CPP = $(SD_UTIL)/*.cpp
UTIL_H = $(ID_UTIL)/*.h


SR_CPP = $(SD_SR)/*.cpp
SR_H = $(ID_SR)/*.h
RS_CPP = $(SD_RS)/*.cpp
RS_H = $(ID_RS)/*.h


DOM_CPP = $(SR_CPP) $(RS_CPP) $(SD_DOMAINS)/*.cpp
DOM_H = $(SR_H) $(RS_H)

ALL_H = $(I_H) $(SOLV_H) $(DOM_H) $(UTIL_H)
ALL_CPP = $(DOM_CPP) $(SOLV_CPP) $(UTIL_CPP)

# Libraries
LIBS = lib/libmdp.a lib/libmdp_domains.a -Llib
LIBS_GUROBI = $(LIBS) -lgurobi60 -Llib

#########################################################################
#                                 TARGETS                               #
#########################################################################

# Compiles the core MDP-LIB library #
libmdp: lib/libmdp.a
lib/libmdp.a: $(OD)/core.a $(OD)/solvers.a
	make $(OD)/core.a
	make $(OD)/solvers.a
	ar rvs libmdp.a $(OD)/core/*.o $(OD)/solvers/*.o
	mkdir -p lib
	mv libmdp.a lib

# Compiles the base (single-objective) solvers
$(OD)/solvers.a: $(S_CPP) $(UTIL_CPP) $(I_H) $(UTIL_H) $(SOLV_CPP) $(SOLV_H) $(ID_DOMAINS)/*.h $(SD_DOMAINS)/*.cpp
	make $(OD)/core.a
	$(CC) $(CFLAGS) $(INCLUDE_CORE) $(ID_DOMAINS)/*.h -c $(SOLV_CPP) \
		$(SD_DOMAINS)/DummyState.cpp $(SD_DOMAINS)/WrapperProblem.cpp
	mkdir -p $(OD_SOLV)
	mv *.o $(OD_SOLV)
	ar rvs $(OD)/solvers.a $(OD_SOLV)/*.o

# Compiles the core classes
$(OD)/core.a: $(S_CPP) $(UTIL_CPP) $(I_H) $(UTIL_H)
	$(CC) $(CFLAGS) $(INCLUDE_CORE) -c $(UTIL_CPP) $(S_CPP) $(UTIL_CPP)
	mkdir -p obj/core
	mv *.o obj/core
	ar rvs $(OD)/core.a $(OD)/core/*.o



domains: lib/libmdp_domains.a
lib/libmdp_domains.a: lib/libmdp.a $(DOM_H) $(DOM_CPP)
	$(CC) $(CFLAGS) $(INCLUDE) -c $(DOM_CPP)
	mkdir -p $(OD_DOMAINS)
	mv *.o $(OD_DOMAINS)
	ar rvs lib/libmdp_domains.a $(OD_DOMAINS)/*.o

		
# Compiles the GUSSP test program
testGussp.out: lib/libmdp.a domains
	$(CC) $(CFLAGS) $(INCLUDE) -o testGussp.out $(TD)/testGUSSP.cpp $(LIBS)
	

.PHONY: clean
clean:
	rm -f $(TD)/*.o
	rm -f *.o
	rm -f $(OD)/*.a
	rm -f $(OD)/*.o
	rm -f $(OD)/core/*.o
	rm -f $(OD)/domains/*.o
	rm -f $(OD)/domains/*.a
	rm -f $(OD)/solvers/*.o
	rm -f $(OD)/solvers/*.a
	rm -f lib/libmdp*.a
