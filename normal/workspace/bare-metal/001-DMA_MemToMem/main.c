#include <stdint.h>
#include <stdio.h>


int main()
{
    while (1)
    {
        /* code */
        printf("hello_world\n");
        for(volatile long i = 0; i < 100000; i++);
    }
}

