/*
 * ibus.h
 *
 *  Created on: Dec 9, 2025
 *      Author: filip
 */


#ifndef IBUS_H_
#define IBUS_H_

#include <stdint.h>
#include <stdbool.h>

#define IBUS_CHANNELS 14

// do podglądu w Watch (albo używaj ibus_read_channel())
extern volatile uint16_t ibus_ch[IBUS_CHANNELS];
extern volatile uint32_t ibus_frames_ok;
extern volatile uint32_t ibus_frames_bad;

void     ibus_init(void);        // start DMA RX na UART1
void     ibus_process(void);     // wołaj w while(1)
uint16_t ibus_read_channel(uint8_t ch);  // 0..13
bool ibus_is_signal_present(void);
uint16_t ibus_dma_ndtr(void);


#endif

