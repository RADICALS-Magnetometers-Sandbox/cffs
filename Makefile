ROOT = ${PWD}

FS = FileSystem
FS_PATH = $(ROOT)/$(FS)
INC = $(FS_PATH)/inc

OBJS = main.o \
	$(FS_PATH)/test/nandfs_tests.o $(FS_PATH)/test/sample_png.o \
	$(FS_PATH)/src/nandfs.o \
	$(FS_PATH)/src/core/nand_core.o $(FS_PATH)/src/core/nand_driver.o

main.o: $(FS_PATH)/test/nandfs_tests.h
	$(CC) -c -I$(FS_PATH)/test main.c

$(OBJS) : $(FS) $(DEPS)

CC = gcc
CFLAGS = -Wall -O -g -I$(INC)

PROG = cffs

.PHONY: all $(FS)
all $(PROG): $(OBJS)
	$(CC) -o $(PROG) $(OBJS)

$(FS):
	$(MAKE) -C $@ INC=$(INC)

clean:
	rm $(OBJS)

