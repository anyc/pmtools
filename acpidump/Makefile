PROG= acpidump
SRCS=	acpidump.c
KERNEL_INCLUDE := ../include
CFLAGS += -Wall -Wstrict-prototypes -Wdeclaration-after-statement -D_LINUX -DDEFINE_ALTERNATE_TYPES -I$(KERNEL_INCLUDE) 

all: acpidump
$(PROG) : $(SRCS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRCS) -o $(PROG)

CLEANFILES= $(PROG)

clean : 
	rm -f $(CLEANFILES) $(patsubst %.c,%.o, $(SRCS)) *~

