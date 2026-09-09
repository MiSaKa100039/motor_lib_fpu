/* Project-owned GNU linker script for the 64 KiB Flash / 8 KiB RAM device. */

ENTRY(Reset_Handler)

_Min_Heap_Size = 0x000;
_Min_Stack_Size = 0x800;

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 8K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

PHDRS
{
    flash PT_LOAD FLAGS(5);
    ram   PT_LOAD FLAGS(6);
}

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH :flash

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP(*(.init))
        KEEP(*(.fini))
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH :flash

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH :flash

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH :flash

    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH :flash

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH :flash

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH :flash

    _sidata = LOADADDR(.data);

    /* 64 位定点状态要求 8 字节对齐，RAM 段的 VMA/LMA 间隙须保持一致。 */
    .data : ALIGN(8)
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(8);
        _edata = .;
    } > RAM AT > FLASH :ram

    .bss (NOLOAD) :
    {
        . = ALIGN(8);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(8);
        _ebss = .;
        __bss_end__ = _ebss;
    } > RAM :ram

    ._user_heap_stack (NOLOAD) :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . += _Min_Heap_Size;
        . += _Min_Stack_Size;
        . = ALIGN(8);
    } > RAM :NONE

    /DISCARD/ :
    {
        *(.note.GNU-stack)
        *(.comment)
    }
}

ASSERT(SIZEOF(.isr_vector) == 192, "Unexpected LCM32F039 vector table size")
ASSERT(_estack == 0x20002000, "Unexpected LCM32F039 stack top")
ASSERT((_ebss + _Min_Heap_Size + _Min_Stack_Size) <= _estack,
       "LCM32F039 RAM overflow: heap and minimum stack do not fit")
