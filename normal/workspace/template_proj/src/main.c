#include <stdint.h>
#include <stdio.h>
// #include "SEGGER_RTT.h"
// #include "nrf52840.h"
// #include "nrf52840_bitfields.h"


void delay(uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
    {
        __asm("nop");
    }
}

int main()
{
    // printf("hello world\n");
    // SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
    // SEGGER_RTT_WriteString(0, "hello world\n");
    while (1)
    {
        /* code */

    }
    
    return 0;
}

