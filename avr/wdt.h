
#ifndef AVR_WDT_H
#define AVR_WDT_H

#define WDTO_1S 6

void wdt_enable(uint8_t timeout);
void wdt_reset();

#endif
