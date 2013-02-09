all: madt

test: madt
	./madt < APIC
madt: madt.c acpi.h tables.c
	$(CC) $(CFLAGS) $(LDFLAGS) madt.c -o madt
clean:
	rm madt
