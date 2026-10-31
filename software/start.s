.global _start
.section .text.init

_start:
    # Initialize Stack Pointer
    la sp, __stack_top  

    # Clear BSS section using standard RISC-V temporary registers
    la t0, _bss_start
    la t1, _bss_end
1:
    bge t0, t1, 2f
    sw zero, 0(t0)
    addi t0, t0, 4
    j 1b
2:
    # Jump to C code
    jal ra, main       

halt:
    j halt