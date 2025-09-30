
/* Entry Point */
ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM)

MEMORY
{
    CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 64K
    RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 64K
    FLASH (rx) : ORIGIN = 0x80000000, LENGTH = 32K
}

SECTIONS
{
    .isr_vector:
    {

    } > 
}

