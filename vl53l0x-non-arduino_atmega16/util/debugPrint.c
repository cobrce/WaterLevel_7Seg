#include <stdint.h>
#include "debugPrint.h"

#define BAUD 115200
#include <util/setbaud.h>

#if DEBUGPRINT_ENABLED


void debugInit() {
    // Init UART
    /* PRR &= ~(1<<PRUSART0); */
    UCSRB |= (1<<TXEN) | (1<<RXEN);
    UBRRH = UBRRH_VALUE;
    UBRRL = UBRRL_VALUE;
    #if USE_2X
        UCSRA |= (1<<U2X);
    #else
        UCSRA &= ~(1<<U2X);
    #endif
}

void debug_putc( uint8_t c ) {
    while(!(UCSRA & (1 << UDRE)));    /* wait for data register empty */
    UDR = c;
}

uint8_t hex2Nibble( uint8_t c ){
    if(  c>='a' && c<='f' ){
        return( c - 'a' + 10 );
    }
    if(  c>='0' && c<='9' ){
        return( c - '0' );
    }
    return 0xFF;
}

// Print string from flash
void debug_str_internal( const char *p ){
    uint8_t c;
    while (( c=pgm_read_byte( p++ ) )){
        debug_putc( c );
        if( c=='\n' ){
            debug_putc('\r');
        }
    }
}

void debug_dec( uint32_t val ){
    char buffer[10];
    char *p = buffer;
    while (val || p == buffer) {
        *(p++) = val % 10;
        val = val / 10;
    }
    while (p != buffer) {
        debug_putc( '0' + *(--p) );
    }
}

void debug_dec_fix( uint32_t val, const uint8_t nFract ){
    uint32_t fractMask = ((1<<nFract)-1);   //For masking the fractional part
    debug_dec( val>>nFract );               //Print the integer part
    debug_putc('.');
    val &= fractMask;                       //Convert to fractional part
    while( val > 0 ) {
        val *= 10;
        debug_putc( '0' + (val>>nFract) );  //Print digit
        val &= fractMask;                   //Convert to fractional part
    }
}

void debug_hex( uint32_t val, uint8_t digits ){
    for (int i = (4*digits)-4; i >= 0; i -= 4)
        debug_putc( "0123456789ABCDEF"[(val >> i) % 16] );
}

// Print a pretty hex-dump on the debug out
void hexDump( uint8_t *buffer, uint16_t nBytes ){
    for( uint16_t i=0; i<nBytes; i++ ){
        if( (nBytes>16) && ((i%16)==0) ){
            debug_str("\n    ");
            debug_hex(i, 4);
            debug_str(": ");
        }
        debug_hex(*buffer++, 2);
        debug_str(" ");
    }
}

#endif
