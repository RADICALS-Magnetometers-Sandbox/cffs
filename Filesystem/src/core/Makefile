CC = gcc
CFLAGS = -Wall -O -g -I$(INC)

nand_core.o: $(INC)/core/nand_core.h $(INC)/core/nand_m79a_lld.h

nand_driver.o: $(INC)/core/nand_m79a_lld.h

all: nand_core.o nand_driver.o

