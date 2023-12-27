BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= riscv64-unknown-elf-
GCC     = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS  = -march=rv32im -mabi=ilp32 -Wall -fPIC -Wa,-adhls=$*.lst
CFLAGS += -Os -std=gnu99 -c

K1986VK025_OBJS := k1986vk025.o
MDR1206FI_215_OBJS := mdr1206fi_215.o
MDR1206FI_217_OBJS := mdr1206fi_217.o

all: k1986vk025_algo.inc \
     mdr1206fi_215_algo.inc \
     mdr1206fi_217_algo.inc

%.o: %.c
	$(GCC) $(CFLAGS) -o $@ $<

k1986vk025_algo.bin: $(K1986VK025_OBJS)
	$(OBJCOPY) -Obinary $< $@

mdr1206fi_215_algo.bin: $(MDR1206FI_215_OBJS)
	$(OBJCOPY) -Obinary $< $@

mdr1206fi_217_algo.bin: $(MDR1206FI_217_OBJS)
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	@echo 'Cleaning Targets and Build Artifacts'
	rm -rf *.inc *.bin *.lst

.PRECIOUS: %.bin

.PHONY: all clean
