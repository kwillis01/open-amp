__DDR_START__ = 0xA2000000;

MEMORY
{
	TCMA (rwx) : ORIGIN = 0, LENGTH = 0x8000
	DDR_0 (rwx) : ORIGIN = __DDR_START__ + 0x100000, LENGTH = 0x1000
	DDR_1 (rwx) : ORIGIN = __DDR_START__ + 0x101000, LENGTH = 0xEFF000
}

SECTIONS
{
	.boot :
	{
		KEEP(*(.isr_vector))
		*(.boot*)
		*(.text.mpu*)
		*(.text.cache*)
		*(.text.boot*)
		*(.text.hwi*)
	} > TCMA

	.text :
	{
		*(.text*)

		KEEP(*(.init))
		KEEP(*(.fini))

		/* .ctors */
		*crtbegin.o(.ctors)
		*crtbegin?.o(.ctors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
		*(SORT(.ctors.*))
		*(.ctors)

		/* .dtors */
		*crtbegin.o(.dtors)
		*crtbegin?.o(.dtors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
		*(SORT(.dtors.*))
		*(.dtors)

		*(.rodata*)

		KEEP(*(.eh_frame*))
	} > DDR_1

	.ARM.extab :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > DDR_1

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > DDR_1
	__exidx_end = .;

	/* To copy multiple ROM to RAM sections,
	 * uncomment .copy.table section and,
	 * define __STARTUP_COPY_MULTIPLE in startup_ARMCMx.S */
	/*
	.copy.table :
	{
		. = ALIGN(4);
		__copy_table_start__ = .;
		LONG (__etext)
		LONG (__data_start__)
		LONG (__data_end__ - __data_start__)
		*(.stack*)
	} > DDR_1

	/* Set stack top to end of RAM, and stack limit move down by
	 * size of stack_dummy section */
	__StackTop = ORIGIN(DDR_1) + LENGTH(DDR_1);
	__StackLimit = __StackTop - SIZEOF(.stack_dummy);
	PROVIDE(__stack = __StackTop);

	/* Check if data + heap + stack exceeds RAM limit */
	ASSERT(__StackLimit >= __HeapLimit, "region TCMA overflowed with stack")

	
}
