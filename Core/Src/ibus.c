#include "ibus.h"
#include "usart.h"
#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdbool.h>

//KANALY IBUS:
//0 - PRAWY DRAZEK POZIOMO
//1 - PRAWY DRAZEK PIONOWO
//2 - LEWY DRAZEK PIONOWO
//3 - LEWY DRAZEK POZIOMO
//4 - VRA - LEWE POKRETLO NA GORZE
//5 - VRB - PRAWE POKRETLO NA GORZE


#define IBUS_DMA_BUF_SZ  64
#define IBUS_FRAME_SZ    32

static uint8_t ibus_dma_buf[IBUS_DMA_BUF_SZ];

volatile uint16_t ibus_ch[IBUS_CHANNELS];
volatile uint32_t ibus_frames_ok = 0;
volatile uint32_t ibus_frames_bad = 0;
volatile uint32_t ibus_last_frame_ms = 0;
static uint16_t ibus_last_ndtr = IBUS_DMA_BUF_SZ;

static uint16_t ibus_checksum(const uint8_t *frame)
{
  uint16_t sum = 0;
  for (int i = 0; i < 30; i++) sum += frame[i];
  return (uint16_t)(0xFFFFu - sum);
}

// Przeszukuje bufor DMA- jak znajdzie poprawna ramke iBUS to aktualizuje ibus_ch[]
static void ibus_try_parse_from_dma(void)
{
  for (int start = 0; start <= (IBUS_DMA_BUF_SZ - IBUS_FRAME_SZ); start++)
  {
    const uint8_t *f = &ibus_dma_buf[start];

    // header iBUS - pomijany
    if (f[0] != 0x20 || f[1] != 0x40) continue;

    uint16_t rx_chk = (uint16_t)f[30] | ((uint16_t)f[31] << 8);
    uint16_t calc   = ibus_checksum(f);

    if (rx_chk != calc)
    {
      ibus_frames_bad++;
      continue;
    }

    for (int ch = 0; ch < IBUS_CHANNELS; ch++)
    {
      int idx = 2 + ch * 2;
      ibus_ch[ch] = (uint16_t)f[idx] | ((uint16_t)f[idx + 1] << 8);
    }

    ibus_frames_ok++;
    ibus_last_frame_ms = HAL_GetTick();
    return;
  }
}

void ibus_init(void)
{
  // wyczysc na start
  memset((void*)ibus_ch, 0, sizeof(ibus_ch));
  ibus_frames_ok = 0;
  ibus_frames_bad = 0;
  ibus_last_frame_ms = 0;
  ibus_last_ndtr = IBUS_DMA_BUF_SZ;

  // Start odbioru iBUS
  HAL_UART_Receive_DMA(&huart1, ibus_dma_buf, IBUS_DMA_BUF_SZ);
}

void ibus_process(void)
{
  uint16_t now_ndtr = ibus_dma_ndtr();

  // Jeśli DMA stoi w tym samym miejscu, nie parsuj w kółko starej ramki.
  // To jest ważne dla failsafe po wypięciu przewodu / zatrzymaniu odbiornika.
  if (now_ndtr == ibus_last_ndtr)
      return;

  ibus_last_ndtr = now_ndtr;
  ibus_try_parse_from_dma();
}

bool ibus_is_signal_present(void)
{
  return (ibus_last_frame_ms != 0 &&
          (HAL_GetTick() - ibus_last_frame_ms) < 100);
}

uint16_t ibus_dma_ndtr(void)
{
  return __HAL_DMA_GET_COUNTER(huart1.hdmarx); // ile jeszcze do końca bufora
}

uint16_t ibus_read_channel(uint8_t ch)
{
  if (ch >= IBUS_CHANNELS) return 0;
  return ibus_ch[ch];
}
