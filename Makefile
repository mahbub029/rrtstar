include ../../common.mk


#-----------------------------------------------------------------------------
# Program
#-----------------------------------------------------------------------------

SMP = $(SMP_BIN_PATH)/example_milcom

SMP_OBJ_COMPILE = milcom.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o 

SMP_OBJ = milcom.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o 


SMP2 = $(SMP_BIN_PATH)/example_milcom

SMP_OBJ_COMPILE2 = milcom.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o 

SMP_OBJ2 = milcom.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o 


#SMP2 = $(SMP_BIN_PATH)/example_standalone_rrtstar_cooperative

#SMP_OBJ_COMPILE2 = standalone_rrtstar_cooperative.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o 

#SMP_OBJ2 = standalone_rrtstar_cooperative.o \
	$(SMP_SRC_PATH)/smp/external_libraries/kdtree/kdtree.o 

#-----------------------------------------------------------------------------
# Make the program
#-----------------------------------------------------------------------------

all: 
	$(MAKE) targets

default: 
	$(MAKE) targets	

targets:  $(SMP) 


$(SMP): $(SMP_OBJ) 
	$(LDXX) $(SMP_OBJ_COMPILE) -o $(SMP) $(LDFLAGS) 

$(SMP): $(SMP_OBJ2) 
	$(LDXX) $(SMP_OBJ_COMPILE2) -o $(SMP2) $(LDFLAGS) 



#-----------------------------------------------------------------------------
# Cleaning
#-----------------------------------------------------------------------------

clean:
	-find ./ -name \*.o -exec rm {} \;
	-find ./ -name \*.gch -exec rm {} \;
