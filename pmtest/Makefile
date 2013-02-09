KERNEL_INCLUDE := /usr/src/linux/include

CC = gcc

CFLAGS := -Wall -fno-strength-reduce -fomit-frame-pointer
CFLAGS := $(CFLAGS) -D__KERNEL__ -DMODULE -I$(KERNEL_INCLUDE)
CFLAGS := $(CFLAGS) -Wall -Wno-unused -Wno-multichar

all: pmtest.o

clean:
	$(RM) *.o
