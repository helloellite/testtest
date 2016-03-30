#------------------------------------------------------------------------------
#	define target veriable, to generate a specific
#	target, modify this veriable.
#------------------------------------------------------------------------------
TARGET = libgp.so

#------------------------------------------------------------------------------
#	compiler and linker options
#------------------------------------------------------------------------------
CC = g++

CFLAGS += -Wall
CFLAGS += -g
CFLAGS += -fPIC
CFLAGS += -std=c++11
CFLAGS += -DUSING_ROBIX4

LFLAGS += -shared
LFLAGS += -fPIC

INCLUDES  += -I./robix4 -I./robix4/protocols

#------------------------------------------------------------------------------
#	%.o file declarations
#------------------------------------------------------------------------------
OBJS += gp_geo.o
OBJS += gp_fun.o

#------------------------------------------------------------------------------
#	support two types of target 'libgp.a' 'libgp.so'
#------------------------------------------------------------------------------
$(TARGET) : $(OBJS)
ifeq ($(TARGET), libgp.a)
	@echo ''
	@echo '*************************COMPILE SUCCESSED*************************'
	@ar rv $(TARGET) $(OBJS)
	@echo '*******************************************************************'
	@echo ''
endif
ifeq ($(TARGET), libgp.so)
	$(CC) -o $(TARGET) $(LFLAGS) $(OBJS) $(LIBS)
	@echo ''
	@echo '*************************COMPILE SUCCESSED!************************'
	@ldd $(TARGET)
	@echo '*******************************************************************'
	@echo ''
endif

#------------------------------------------------------------------------------
#	for all .cpp & .c file, generate corresponding.o
#------------------------------------------------------------------------------
%.o:%.cpp
	$(CC) -c $(CMARCO) $(CFLAGS) $(INCLUDES) $< -o $@

%.o:%.c
	$(CC) -c $(CMARCO) $(CFLAGS) $(INCLUDES) $< -o $@

#------------------------------------------------------------------------------
#	for 'make clean' & make 'install'
#------------------------------------------------------------------------------
.PHONY: clean
clean:
	@rm -rf *.o $(TARGET)

.PHONY: install
install:
	@rm -f ~/RBX4/URobix/lib/$(TARGET)
	@install -m 777 $(TARGET) ~/RBX4/URobix/lib
	@stat ~/RBX4/URobix/lib/$(TARGET)
