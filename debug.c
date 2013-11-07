/*
 * Copyright 2013, winocm. <winocm@icloud.com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   If you are going to use this software in any form that does not involve
 *   releasing the source to this project or improving it, let me know beforehand.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "genboot.h"
#include <stdarg.h>

/* Uart stuff */
#define AMBA_UART_DR(base)      (*(volatile unsigned char *)((base) + 0x00))
#define AMBA_UART_LCRH(base)    (*(volatile unsigned char *)((base) + 0x2c))
#define AMBA_UART_CR(base)      (*(volatile unsigned char *)((base) + 0x30))
#define AMBA_UART_FR(base)      (*(volatile unsigned char *)((base) + 0x18))
#define REALVIEW_PBA8_SDRAM6_BASE               0x70000000  /* SDRAM bank 6 256MB */
#define REALVIEW_PBA8_SDRAM7_BASE               0x80000000  /* SDRAM bank 7 256MB */
#define REALVIEW_PBA8_UART0_BASE                0x10009000  /* UART 0 */

#define UART_FR_TXFE (1 << 7)
#define UART_FR_TXFF (1 << 5)

#define UART_FR_RXFE (1 << 4)
#define UART_FR_RXFF (1 << 6)

#define barrier()               __asm__ __volatile__("": : :"memory");

#define GPFSEL1 0x20200004
#define GPSET0  0x2020001C
#define GPCLR0  0x20200028
#define GPPUD       0x20200094
#define GPPUDCLK0   0x20200098



#define UART0_BASE   0x20201000
#define UART0_DR     (UART0_BASE+0x00)
#define UART0_RSRECR (UART0_BASE+0x04)
#define UART0_FR     (UART0_BASE+0x18)
#define UART0_ILPR   (UART0_BASE+0x20)
#define UART0_IBRD   (UART0_BASE+0x24)
#define UART0_FBRD   (UART0_BASE+0x28)
#define UART0_LCRH   (UART0_BASE+0x2C)
#define UART0_CR     (UART0_BASE+0x30)
#define UART0_IFLS   (UART0_BASE+0x34)
#define UART0_IMSC   (UART0_BASE+0x38)
#define UART0_RIS    (UART0_BASE+0x3C)
#define UART0_MIS    (UART0_BASE+0x40)
#define UART0_ICR    (UART0_BASE+0x44)
#define UART0_DMACR  (UART0_BASE+0x48)
#define UART0_ITCR   (UART0_BASE+0x80)
#define UART0_ITIP   (UART0_BASE+0x84)
#define UART0_ITOP   (UART0_BASE+0x88)
#define UART0_TDR    (UART0_BASE+0x8C)

typedef		unsigned int		uint32;
extern void dummy(unsigned int);

uint32 GET32( uint32 addr ) {
	// Create a pointer to our location in memory.
	volatile uint32* ptr = (volatile uint32*)( addr );
	// Return the value.
	return (uint32)(*ptr);
}

void PUT32( uint32 addr, uint32 value ) {
	// Create a pointer to our location in memory.
	volatile uint32* ptr = (volatile uint32*)( addr );
	// Set the value.
	*ptr = value;
}

/**
 * uart_putc
 *
 * Put a character to the system console.
 */
uint32_t uart_base = 0x10009000;
static int inited_printf = 1;
void uart_putchar(int c)
{
    if (!inited_printf)
        return;

    if (c == '\n')
        uart_putchar('\r');

	while(1)
	{
		if((GET32(UART0_FR)&0x20)==0) break;
	}
	PUT32(UART0_DR,c);

}

/**
 * uart_getc
 *
 * Get a character from system input.
 */
int uart_getchar(void)
{
	for (int i = 0; i < 10; i++){
		if((GET32(UART0_FR)&0x10)==0) break;
	}
	return(GET32(UART0_DR));
}

static void putc_wrapper(void *p, char c)
{
    uart_putchar(c);
}

/**
 * init_debug
 *
 * Start debugging subsystems.
 */
void uart_init ( void )
{
	unsigned int ra;
	
	PUT32(UART0_CR,0);
	
	ra=GET32(GPFSEL1);
	ra&=~(7<<12); //gpio14
	ra|=4<<12;    //alt0
	ra&=~(7<<15); //gpio15
	ra|=4<<15;    //alt0
	PUT32(GPFSEL1,ra);
	
	PUT32(GPPUD,0);
	for(ra=0;ra<150;ra++) dummy(ra);
	PUT32(GPPUDCLK0,(1<<14)|(1<<15));
	for(ra=0;ra<150;ra++) dummy(ra);
	PUT32(GPPUDCLK0,0);
	
	PUT32(UART0_ICR,0x7FF);
	PUT32(UART0_IBRD,1);
	PUT32(UART0_FBRD,40);
	PUT32(UART0_LCRH,0x70);
	PUT32(UART0_CR,0x301);
}

void init_debug(void)
{
	uart_init();
    init_printf(NULL, putc_wrapper);
    printf("debug_init()\n");
}

void __assert_func(const char *file, int line, const char *method,
                   const char *expression)
{
    panic("Assertion failed in file %s, line %d. (%s)\n",
          file, line, expression);
    while (1) ;
}

/**
 * panic
 *
 * Halt the system and explain why.
 */
#undef panic
void panic(const char *panicStr, ...)
{
    void *caller = __builtin_return_address(0);

    /* Prologue */
    printf("panic(caller 0x%08x): ", caller);

    /* Epilogue */
    va_list valist;
    va_start(valist, panicStr);
    vprintf((char *)panicStr, valist);
    va_end(valist);

    /* We are hanging here. */
    printf("\npanic: we are hanging here...\n");

    /* Halt */
    _locore_halt_system();
}
