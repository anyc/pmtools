#define LENB

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

#define NULL 0
#define	EINVAL 22
#define CONFIG_ACPI_BOOT

#include "./actbl.h"
#include "./acpi.h"

//#include <linux/kernel.h>
#define KERN_INFO ""
#define KERN_ERR ""
#define KERN_WARNING ""

#define printk printf

//#define INPUT_FILE "madt.dat"
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <stdio.h>
//#include <sys/mman.h>
#include <stdio.h> // fread
#include <stdlib.h> // malloc

#include "./tables.c"

int verbose = 0;
/*
/* read standard input
 * write decoded madt to standard output
 */
get_next_entry(acpi_table_entry_header * entry_header)
{
	size_t retval;

	if (verbose) printf("reading %ld byte entry header\n", sizeof(acpi_table_entry_header));
	retval = fread((void *)entry_header, sizeof(acpi_table_entry_header), 1, stdin);
	return retval;
}

u8	buffer[1024];

main()
{
	size_t retval;
	struct acpi_table_madt *madt_header;
	acpi_table_entry_header *entry_header;
	unsigned int bytes_read, csum;
	u32 expected_length;
 	madt_header = (struct acpi_table_madt *)buffer;

	bytes_read = csum = 0;

	if (verbose) printf("reading %ld madt header\n", sizeof(struct acpi_table_madt));

	retval = fread((void *)buffer, sizeof(struct acpi_table_madt), 1, stdin);
	if (retval != 1) {
		perror("fread");
		exit(1);
	}
	expected_length = madt_header->header.length;

	if (verbose) printf("header.length %d\n", madt_header->header.length);

	acpi_table_print((void*)&(buffer[bytes_read]), 0);

	bytes_read = sizeof(struct acpi_table_madt);

	while (get_next_entry((acpi_table_entry_header *)&buffer[bytes_read]) == 1)
	{
		int read_length;

		entry_header = (acpi_table_entry_header *)&buffer[bytes_read];
		if (verbose) printf("type %d length %d\n", entry_header->type, entry_header->length);
		bytes_read += sizeof(acpi_table_entry_header);

		read_length = entry_header->length - sizeof(acpi_table_entry_header);
		if (read_length <= 0) {
			printf("bad length %d\n", read_length);
			continue;
		}
		retval = fread((void *)&buffer[bytes_read], read_length, 1, stdin);
		if (retval != 1) {
			perror("fread failed!");
			goto done;
		}
		bytes_read += read_length;

		acpi_table_print_madt_entry (entry_header);
	}

done:
	csum = acpi_table_compute_checksum((void *)buffer, bytes_read); 


	if (bytes_read == expected_length)
		printf("Length %d OK\n", bytes_read);
	else
		printf("Length ERROR, read %d, expected %d\n",
			bytes_read, expected_length);

	if (csum == 0)
		printf("Checksum OK\n");
	else
		printf("Checksum 0x%x != 0; 0x%x in header ERROR\n", csum,
			 madt_header->header.checksum);

	return 0;
}

