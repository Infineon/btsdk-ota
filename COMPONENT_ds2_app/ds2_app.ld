OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
MEMORY
{
	ram (rwx) : ORIGIN = 0x20FAD8, LENGTH = 0x60128
	xip_section (rx) : ORIGIN = 0x53F000, LENGTH = 0x1000
}
SECTIONS
{
	.app_xip_area : ALIGN (4)
	{
		CREATE_OBJECT_SYMBOLS
		KEEP(*(.rodata.ds2))
		KEEP(*(.text.ds2))
		*(.rodata)
		*(.rodata.*)
		*(.constdata*)
		*(.text)
		*(.text.*)
		*(.gnu.linkonce.t.*)
		*(.gnu.linkonce.r.*)
	} >xip_section
	.data : ALIGN (4)
	{
		CREATE_OBJECT_SYMBOLS
		*(.data)
		*(.data.*)
		*(.gnu.linkonce.d.*)
		*(.init_patch_table)
	} >ram
	.bss : ALIGN (4)
	{
		CREATE_OBJECT_SYMBOLS
		*(.bss)
		*(.bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
	} >ram
}
