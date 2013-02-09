/******************************************************************************
 *
 * Name: actbl.h - Table data structures defined in ACPI specification
 *
 *****************************************************************************/

/*
 * Copyright (C) 2000 - 2005, R. Byron Moore
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    substantially similar to the "NO WARRANTY" disclaimer below
 *    ("Disclaimer") and any redistribution must be conditioned upon
 *    including a substantially similar Disclaimer requirement for further
 *    binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES.
 */

#ifndef __ACTBL_H__
#define __ACTBL_H__

/*
 * ACPI Boot Arch Flags
 */

#define BAF_LEGACY_DEVICES              0x0001
#define BAF_8042_KEYBOARD_CONTROLLER    0x0002

#define GL_OWNED                0x02	/* Ownership of global lock is bit 1 */

#pragma pack(1)

/*
 * ACPI Version-independent tables
 *
 */

struct acpi_rsdp_descriptor {	/* Root System Descriptor Pointer */
	char signature[8];	/* ACPI signature, contains "RSD PTR " */
	u8 checksum;		/* ACPI 1.0 checksum */
	char oem_id[6];		/* OEM identification */
	u8 revision;		/* Must be (0) for ACPI 1.0 or (2) for ACPI 2.0+ */
	u32 rsdt_physical_address;	/* 32-bit physical address of the RSDT */
	u32 length;		/* XSDT Length in bytes, including header */
	u64 xsdt_physical_address;	/* 64-bit physical address of the XSDT */
	u8 extended_checksum;	/* Checksum of entire table (ACPI 2.0) */
	char reserved[3];	/* Reserved, must be zero */
};

struct acpi_table_header {	/* ACPI common table header */
	char signature[4];	/* ASCII table signature */
	u32 length;		/* Length of table in bytes, including this header */
	u8 revision;		/* ACPI Specification minor version # */
	u8 checksum;		/* To make sum of entire table == 0 */
	char oem_id[6];		/* ASCII OEM identification */
	char oem_table_id[8];	/* ASCII OEM table identification */
	u32 oem_revision;	/* OEM revision number */
	char asl_compiler_id[4];	/* ASCII ASL compiler vendor ID */
	u32 asl_compiler_revision;	/* ASL compiler version */
};

/*
 * Prefered Power Management Profiles
 */

enum acpi_prefered_pm_profiles {
	PM_UNSPECIFIED = 0,
	PM_DESKTOP = 1,
	PM_MOBILE = 2,
	PM_WORKSTATION = 3,
	PM_ENTERPRISE_SERVER = 4,
	PM_SOHO_SERVER = 5,
	PM_APPLIANCE_PC = 6
};

/*
 * ACPI 2.0 Firmware ACPI Control Structure (FACS)
 */
struct acpi_facs_descriptor {
	char signature[4];	/* ASCII table signature */
	u32 length;		/* Length of structure, in bytes */
	u32 hardware_signature;	/* Hardware configuration signature */
	u32 firmware_waking_vector;	/* 32-bit physical address of the Firmware Waking Vector. */
	u32 global_lock;	/* Global Lock used to synchronize access to shared hardware resources */
	u32 flags;
	u64 xfirmware_waking_vector;	/* 64-bit physical address of the Firmware Waking Vector. */
	u8 version;		/* Version of this table */
	u8 reserved3[31];	/* Reserved, must be zero */
};

	/* Flags (32 bits) */
	/* 00:    S4BIOS support is present */
	/* 01-07: Reserved, must be zero */
	/* 08-31: Reserved, must be zero */

static inline int acpi_facs_s4_bios_present(u32 flags)
{
	return flags & 0x00000001;
}

/*
 * ACPI 2.0+ Generic Address Structure (GAS)
 */

struct acpi_generic_address {
	u8 address_space_id;	/* Address space where struct or register exists. */
	u8 register_bit_width;	/* Size in bits of given register */
	u8 register_bit_offset;	/* Bit offset within the register */
	u8 access_width;	/* Minimum Access size (ACPI 3.0) */
	u64 address;		/* 64-bit address of struct or register */
};

/*
 * ACPI 2.0+ Fixed ACPI Description Table (FADT)
 */

struct acpi_fadt_descriptor {
	struct acpi_table_header header;	/* ACPI common table header */
	u32 firmware_ctrl;	/* 32-bit physical address of FACS */
	u32 dsdt;		/* 32-bit physical address of DSDT */
	u8 model;		/* System Interrupt Model isn't used in ACPI 2.0 */
	u8 prefer_PM_profile;	/* Conveys preferred power management profile to OSPM. */
	u16 sci_int;		/* System vector of SCI interrupt */
	u32 smi_cmd;		/* Port address of SMI command port */
	u8 acpi_enable;		/* Value to write to smi_cmd to enable ACPI */
	u8 acpi_disable;	/* Value to write to smi_cmd to disable ACPI */
	u8 S4bios_req;		/* Value to write to SMI CMD to enter S4BIOS state */
	u8 pstate_cnt;		/* Processor performance state control */
	u32 pm1a_evt_blk;	/* Port address of Power Mgt 1a acpi_event Reg Blk */
	u32 pm1b_evt_blk;	/* Port address of Power Mgt 1b acpi_event Reg Blk */
	u32 pm1a_cnt_blk;	/* Port address of Power Mgt 1a Control Reg Blk */
	u32 pm1b_cnt_blk;	/* Port address of Power Mgt 1b Control Reg Blk */
	u32 pm2_cnt_blk;	/* Port address of Power Mgt 2 Control Reg Blk */
	u32 pm_tmr_blk;		/* Port address of Power Mgt Timer Ctrl Reg Blk */
	u32 gpe0_blk;		/* Port addr of General Purpose acpi_event 0 Reg Blk */
	u32 gpe1_blk;		/* Port addr of General Purpose acpi_event 1 Reg Blk */
	u8 pm1_evt_len;		/* Byte length of ports at pm1_x_evt_blk */
	u8 pm1_cnt_len;		/* Byte length of ports at pm1_x_cnt_blk */
	u8 pm2_cnt_len;		/* Byte Length of ports at pm2_cnt_blk */
	u8 pm_tm_len;		/* Byte Length of ports at pm_tm_blk */
	u8 gpe0_blk_len;	/* Byte Length of ports at gpe0_blk */
	u8 gpe1_blk_len;	/* Byte Length of ports at gpe1_blk */
	u8 gpe1_base;		/* Offset in gpe model where gpe1 events start */
	u8 cst_cnt;		/* Support for the _CST object and C States change notification. */
	u16 plvl2_lat;		/* Worst case HW latency to enter/exit C2 state */
	u16 plvl3_lat;		/* Worst case HW latency to enter/exit C3 state */
	u16 flush_size;		/* Number of flush strides that need to be read */
	u16 flush_stride;	/* Processor's memory cache line width, in bytes */
	u8 duty_offset;		/* Processor's duty cycle index in processor's P_CNT reg */
	u8 duty_width;		/* Processor's duty cycle value bit width in P_CNT register. */
	u8 day_alrm;		/* Index to day-of-month alarm in RTC CMOS RAM */
	u8 mon_alrm;		/* Index to month-of-year alarm in RTC CMOS RAM */
	u8 century;		/* Index to century in RTC CMOS RAM */
	u16 iapc_boot_arch;	/* IA-PC Boot Architecture Flags. See Table 5-10 for description */
	u8 reserved2;		/* Reserved, must be zero */
	u32 flags;
	struct acpi_generic_address reset_register;	/* Reset register address in GAS format */
	u8 reset_value;		/* Value to write to the reset_register port to reset the system */
	u8 reserved4[3];	/* These three bytes must be zero */
	u64 xfirmware_ctrl;	/* 64-bit physical address of FACS */
	u64 Xdsdt;		/* 64-bit physical address of DSDT */
	struct acpi_generic_address xpm1a_evt_blk;	/* Extended Power Mgt 1a acpi_event Reg Blk address */
	struct acpi_generic_address xpm1b_evt_blk;	/* Extended Power Mgt 1b acpi_event Reg Blk address */
	struct acpi_generic_address xpm1a_cnt_blk;	/* Extended Power Mgt 1a Control Reg Blk address */
	struct acpi_generic_address xpm1b_cnt_blk;	/* Extended Power Mgt 1b Control Reg Blk address */
	struct acpi_generic_address xpm2_cnt_blk;	/* Extended Power Mgt 2 Control Reg Blk address */
	struct acpi_generic_address xpm_tmr_blk;	/* Extended Power Mgt Timer Ctrl Reg Blk address */
	struct acpi_generic_address xgpe0_blk;	/* Extended General Purpose acpi_event 0 Reg Blk address */
	struct acpi_generic_address xgpe1_blk;	/* Extended General Purpose acpi_event 1 Reg Blk address */
};

	/* Flags (32 bits) */
	/* 00:    The wbinvd instruction works properly */
static inline int acpi_fadt_wb_invd(u32 flags)
{
	return flags & 1;
}

	/* 01:    The wbinvd flushes but does not invalidate */
static inline int acpi_fadt_wb_invd_flush(u32 flags)
{
	return flags & (1 << 1);
}

/* 02:    All processors support C1 state */
static inline int acpi_fadt_proc_c1(u32 flags)
{
	return flags & (1 << 2);
}

/* 03:    C2 state works on MP system */
static inline int acpi_fadt_plvl2_up(u32 flags)
{
	return flags & (1 << 3);
}

/* 04:    Power button is handled as a generic feature */
static inline int acpi_fadt_pwr_button(u32 flags)
{
	return flags & (1 << 4);
}

/* 05:    Sleep button is handled as a generic feature, or  not present */
static inline int acpi_fadt_sleep_button(u32 flags)
{
	return flags & (1 << 5);
}

/* 06:    RTC wakeup stat not in fixed register space */
static inline int acpi_fadt_fixed_rTC(u32 flags)
{
	return flags & (1 << 6);
}

/* 07:    RTC wakeup stat not possible from S4 */
static inline int acpi_fadt_rtcs4(u32 flags)
{
	return flags & (1 << 7);
}

/* 08:    tmr_val is 32 bits 0=24-bits */
static inline int acpi_fadt_tmr_val_ext(u32 flags)
{
	return flags & (1 << 8);
}

/* 09:    Docking supported */
static inline int acpi_fadt_dock_cap(u32 flags)
{
	return flags & (1 << 9);
}

/* 10:    System reset via the FADT RESET_REG supported */
static inline int acpi_fadt_reset_reg_sup(u32 flags)
{
	return flags & (1 << 10);
}

/* 11:    No internal expansion capabilities and case is  sealed */
static inline int acpi_fadt_sealed_case(u32 flags)
{
	return flags & (1 << 11);
}

/* 12:    No local video capabilities or local input devices */
static inline int acpi_fadt_headless(u32 flags)
{
	return flags & (1 << 12);
}

/* 13:    Must execute native instruction after writing  SLP_TYPx register */
static inline int acpi_fadt_cpu_sw_sleep(u32 flags)
{
	return flags & (1 << 13);
}

/* 14:    System supports PCIEXP_WAKE (STS/EN) bits (ACPI 3.0) */
static inline int acpi_fadt_pci_exp_wak(u32 flags)
{
	return flags & (1 << 14);
}

/* 15:    OSPM should use platform-provided timer (ACPI 3.0) */
static inline int acpi_fadt_use_platform_clock(u32 flags)
{
	return flags & (1 << 15);
}

/* 16:    Contents of RTC_STS valid after S4 wake (ACPI 3.0) */
static inline int acpi_fadt_s4_rtc_sts_valid(u32 flags)
{
	return flags & (1 << 16);
}

/* 17:    System is compatible with remote power on (ACPI 3.0) */
static inline int acpi_fadt_remote_power_on_capable(u32 flags)
{
	return flags & (1 << 17);
}

/* 18:    All local APICs must use cluster model (ACPI 3.0) */
static inline int acpi_fadt_force_apic_cluster_model(u32 flags)
{
	return flags & (1 << 18);
}

/* 19:    all local x_aPICs must use physical dest mode (ACPI 3.0) */
static inline int acpi_fadt_force_apic_physical_destination_mode(u32 flags)
{
	return flags & (1 << 19);
}

/* ECDT - Embedded Controller Boot Resources Table */

struct acpi_ec_boot_resources {
	struct acpi_table_header header;
	struct acpi_generic_address ec_control;	/* Address of EC command/status register */
	struct acpi_generic_address ec_data;	/* Address of EC data register */
	u32 uid;		/* Unique ID - must be same as the EC _UID method */
	u8 gpe_bit;		/* The GPE for the EC */
	u8 ec_id[1];		/* Full namepath of the EC in the ACPI namespace */
};

/* SRAT - System Resource Affinity Table */

struct acpi_static_resource_alloc {
	u8 type;
	u8 length;
	u8 proximity_domain_lo;
	u8 apic_id;
	u32 flags;
	u8 local_sapic_eid;
	u8 proximity_domain_hi[3];
	u32 reserved4;		/* Reserved, must be zero */
};

 /* 00:    Use affinity structure */
static inline int acpi_srat_enabled(u32 flags)
{
	return flags && 1;
}

struct memory_affinity {
	u8 type;
	u8 length;
	u32 proximity_domain;
	u16 reserved3;
	u64 base_address;
	u64 address_length;
	u32 reserved4;
	u32 flags;
	u64 reserved6;		/* Reserved, must be zero */
};

/* 00:    Use affinity structure */
static inline int acpi_memory_affinity_enabled(u32 flags)
{
	return flags && 1;
}

/* 01:    Memory region is hot pluggable */
static inline int acpi_memory_affinity_hot_pluggable(u32 flags)
{
	return flags && 2;
}

/* 02:    Memory is non-volatile */
static inline int acpi_memory_affinity_non_volatile(u32 flags)
{
	return flags && 4;
}

struct acpi_system_resource_affinity {
	struct acpi_table_header header;
	u32 reserved1;		/* Must be value '1' */
	u64 reserved2;		/* Reserved, must be zero */
};

/* SLIT - System Locality Distance Information Table */

struct acpi_system_locality_info {
	struct acpi_table_header header;
	u64 locality_count;
	u8 entry[1][1];
};

/*
 * MADT values and structures
 */

/* Values for MADT PCATCompat */

#define DUAL_PIC                0
#define MULTIPLE_APIC           1

/* Master MADT */

struct acpi_multiple_apic_table {
	struct acpi_table_header header;	/* ACPI common table header */
	u32 local_apic_address;	/* Physical address of local APIC */
	u32 flags;
};

/* 00:    System also has dual 8259s */
static inline int acpi_madt_PCATcompat(u32 flags)
{
	return flags && 1;
}

enum acpi_apic_type {
	APIC_PROCESSOR = 0,
	APIC_IO = 1,
	APIC_XRUPT_OVERRIDE = 2,
	APIC_NMI = 3,
	APIC_LOCAL_NMI = 4,
	APIC_ADDRESS_OVERRIDE = 5,
	APIC_IO_SAPIC = 6,
	APIC_LOCAL_SAPIC = 7,
	APIC_XRUPT_SOURCE = 8,
	APIC_RESERVED = 9	/* 9 and greater are reserved */
};

/*
 * MADT sub-structures (Follow MULTIPLE_APIC_DESCRIPTION_TABLE)
 */

/* Common flag definitions (16 bits each) */

/*
	MPS INTI Flags
	Bits 00-01: Polarity of APIC I/O input signals
	Bits 02-03: Trigger mode of APIC input signals
	Bits 04-15: Reserved, must be zero
*/

enum acpi_polarity_type {
	POLARITY_CONFORMS = 0,
	POLARITY_ACTIVE_HIGH = 1,
	POLARITY_RESERVED = 2,
	POLARITY_ACTIVE_LOW = 3
};

enum acpi_trigger_type {
	TRIGGER_CONFORMS = 0,
	TRIGGER_EDGE = 1,
	TRIGGER_RESERVED = 2,
	TRIGGER_LEVEL = 3
};

static inline enum acpi_polarity_type acpi_get_polarity_type(u16 mps_inti_flags)
{
	return mps_inti_flags & 0x0003;
}

static inline enum acpi_trigger_type acpi_get_trigger_type(u16 mps_inti_flags)
{
	return mps_inti_flags & 0x0012;
}

/* Sub-structures for MADT */

struct acpi_madt_processor_apic {
	enum acpi_apic_type type:8;
	u8 length;
	u8 processor_id;	/* ACPI processor id */
	u8 local_apic_id;	/* Processor's local APIC id */
	u16 flags;
};

/*
	Local Apic Flags
	Bit 1: Processor is usable if set
	Bit 2-15: Reserved, must be zero
*/

static inline int acpi_madt_processor_apic_usable(u16 flags)
{
	return flags & 1;
}

struct acpi_madt_io_apic {
	enum acpi_apic_type type:8;
	u8 length;
	u8 io_apic_id;		/* I/O APIC ID */
	u8 reserved;		/* Reserved - must be zero */
	u32 address;		/* APIC physical address */
	u32 interrupt;		/* Global system interrupt where INTI
				 * lines start */
};

struct acpi_madt_interrupt_override {
	enum acpi_apic_type type:8;
	u8 length;
	u8 bus;			/* 0 - ISA */
	u8 source;		/* Interrupt source (IRQ) */
	u32 interrupt;		/* Global system interrupt */
	u16 mps_inti_flags;
};

struct acpi_madt_nmi_source {
	enum acpi_apic_type type:8;
	u8 length;
	u16 mps_inti_flags;
	u32 interrupt;		/* Global system interrupt */
};

struct acpi_madt_local_apic_nmi {
	enum acpi_apic_type type:8;
	u8 length;
	u8 processor_id;	/* ACPI processor id */
	u16 mps_inti_flags;
	u8 lint;		/* LINTn to which NMI is connected */
};

struct acpi_madt_address_override {
	enum acpi_apic_type type:8;
	u8 length;
	u16 reserved;		/* Reserved, must be zero */
	u64 address;		/* APIC physical address */
};

struct acpi_madt_io_sapic {
	enum acpi_apic_type type:8;
	u8 length;
	u8 io_sapic_id;		/* I/O SAPIC ID */
	u8 reserved;		/* Reserved, must be zero */
	u32 interrupt_base;	/* Glocal interrupt for SAPIC start */
	u64 address;		/* SAPIC physical address */
};

struct acpi_madt_local_sapic {
	enum acpi_apic_type type:8;
	u8 length;
	u8 processor_id;	/* ACPI processor id */
	u8 local_sapic_id;	/* SAPIC ID */
	u8 local_sapic_eid;	/* SAPIC EID */
	u8 reserved[3];		/* Reserved, must be zero */
	u16 local_apic_flags;
	u32 processor_uID;	/* Numeric UID - ACPI 3.0 */
	char processor_uIDstring[1];	/* String UID  - ACPI 3.0 */
};

struct acpi_madt_interrupt_source {
	enum acpi_apic_type type:8;
	u8 length;
	u16 mps_inti_flags;
	u8 interrupt_type;	/* 1=PMI, 2=INIT, 3=corrected */
	u8 processor_id;	/* Processor ID */
	u8 processor_eid;	/* Processor EID */
	u8 io_sapic_vector;	/* Vector value for PMI interrupts */
	u32 interrupt;		/* Global system interrupt */
	u32 flags;		/* Interrupt Source Flags */
};

/*
 * Smart Battery
 */

struct acpi_smart_battery_table {
	struct acpi_table_header header;
	u32 warning_level;
	u32 low_level;
	u32 critical_level;
};

/*
 * High performance timer
 */

struct acpi_hpet_table {
	struct acpi_table_header header;
	u32 hardware_id;
	struct acpi_generic_address base_address;
	u8 hpet_number;
	u16 clock_tick;
	u8 attributes;
};

#pragma pack()

/*
 * ACPI Table information.  We save the table address, length,
 * and type of memory allocation (mapped or allocated) for each
 * table for 1) when we exit, and 2) if a new table is installed
 */
#define ACPI_MEM_NOT_ALLOCATED  0
#define ACPI_MEM_ALLOCATED      1
#define ACPI_MEM_MAPPED         2

/* Definitions for the Flags bitfield member of struct acpi_table_support */

#define ACPI_TABLE_SINGLE       0x00
#define ACPI_TABLE_MULTIPLE     0x01
#define ACPI_TABLE_EXECUTABLE   0x02

#define ACPI_TABLE_ROOT         0x00
#define ACPI_TABLE_PRIMARY      0x10
#define ACPI_TABLE_SECONDARY    0x20
#define ACPI_TABLE_ALL          0x30
#define ACPI_TABLE_TYPE_MASK    0x30

/* Data about each known table type */

/*
 *  Values for description table header signatures
 */

#define RSDP_SIG                "RSD PTR "	/* RSDT Pointer signature */
#define APIC_SIG                "APIC"	/* Multiple APIC Description Table */
#define DSDT_SIG                "DSDT"	/* Differentiated System Description Table */
#define FADT_SIG                "FACP"	/* Fixed ACPI Description Table */
#define FACS_SIG                "FACS"	/* Firmware ACPI Control Structure */
#define PSDT_SIG                "PSDT"	/* Persistent System Description Table */
#define RSDT_SIG                "RSDT"	/* Root System Description Table */
#define XSDT_SIG                "XSDT"	/* Extended  System Description Table */
#define SSDT_SIG                "SSDT"	/* Secondary System Description Table */
#define SBST_SIG                "SBST"	/* Smart Battery Specification Table */
#define SPIC_SIG                "SPIC"	/* IOSAPIC table */
#define BOOT_SIG                "BOOT"	/* Boot table */

struct acpi_table_support {
	char signature[8];
	void **global_ptr;
	u8 sig_length;
	u8 flags;
};

#endif				/* __ACTBL_H__ */
