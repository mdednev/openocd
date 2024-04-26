BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= riscv64-unknown-elf-
GCC     = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS  = -march=rv32im -mabi=ilp32 -Wall -fPIC -Wa,-adhls=$*.lst
CFLAGS += -Os -std=gnu99 -c

all: mdr32f02fi_flash_write.inc \
     mdr1206fi_flash_write.inc  \
     mdr1206afi_flash_write.inc

%.o: %.c
	$(GCC) $(CFLAGS) -o $@ $<

%.bin: %.o
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	@echo 'Cleaning Targets and Build Artifacts'
	rm -rf *.o *.lst *.bin

.PRECIOUS: %.o %.bin

.PHONY: all clean
