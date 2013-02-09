PROG=	acpixtract
SRCS=	acpixtract.c

CFLAGS+= -Wall -O2 -D_LINUX -DACPI_APPLICATION -Wstrict-prototypes -I../../include 

all: $(PROG)

$(PROG) : $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(PROG)

CLEANFILES= $(PROG)

clean : 
	rm -f $(CLEANFILES) $(patsubst %.c,%.o, $(SRCS))
