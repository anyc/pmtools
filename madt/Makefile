test: madt
	./madt < APIC
madt: madt.c acpi.h tables.c
	cc -o madt madt.c
clean:
	rm madt
