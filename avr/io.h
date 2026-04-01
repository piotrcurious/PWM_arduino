
#ifndef AVR_IO_H
#define AVR_IO_H

#include <stdint.h>

#define _BV(bit) (1 << (bit))

extern uint8_t TCCR1A;
extern uint8_t TCCR1B;
extern uint16_t ICR1;
extern uint16_t OCR1A;
extern uint16_t OCR1B;

#define COM1A1 7
#define COM1B1 5
#define WGM11 1
#define WGM10 0
#define WGM13 4
#define WGM12 3
#define CS12 2
#define CS11 1
#define CS10 0

#define F_CPU 16000000L

#endif
