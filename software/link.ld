MEMORY
{
    BRAM (rwx) : ORIGIN = 0x00000000, LENGTH = 16384
}
SECTIONS
{
    .text : { *(.text.init) *(.text*) } > BRAM
    .rodata : { *(.rodata*) } > BRAM
    .data : { *(.data*) *(.sdata*) } > BRAM
    .bss : { 
        _bss_start = .;
        *(.bss*) 
        *(.sbss*) 
        _bss_end = .;
    } > BRAM
    
    __stack_top = ORIGIN(BRAM) + LENGTH(BRAM);
}