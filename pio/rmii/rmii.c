/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// PIO logic analyser example
//
// This program captures samples from a group of pins, at a fixed rate, once a
// trigger condition is detected (level condition on one pin). The samples are
// transferred to a capture buffer using the system DMA.
//
// 1 to 32 pins can be captured, at a sample rate no greater than system clock
// frequency.

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"

// Some logic to analyse:
#include "hardware/structs/pwm.h"

#include "rmii.pio.h"

const uint CAPTURE_PIN_COUNT = 4;
const uint CAPTURE_N_SAMPLES = 2048; //96;

void rmii_rx_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(dma_chan, &c,
        capture_buf,        // Destinatinon pointer
        &pio->rxf[sm],      // Source pointer
        capture_size_words, // Number of transfers
        true                // Start immediately
    );

    pio_sm_set_enabled(pio, sm, true);
}

#define main x
#define SCNx8 "c"
#include "Ethernet-CRC32/crc32.c"
#undef main

void print_capture_buf(const uint32_t *buf, uint pin_count, uint32_t n_samples) {
    // Display the capture buffer in text form, like this:
    // 00: __--__--__--__--__--__--
    // 01: ____----____----____----
    static const char* names[] = { "RX0", "RX1", "CRS", "CLK" };
    static int cnt = 0;
    printf("Capture %d:\n", cnt++);
    for (int pin = 0; pin < pin_count; ++pin) {
        printf("%s: ", names[pin]);
        for (int sample = 0; sample < n_samples; ++sample) {
            uint bit_index = pin + sample * pin_count;
            bool level = !!(buf[bit_index / 32] & 1u << (bit_index % 32));
            printf(level ? "-" : "_");
        }
        printf("\n");
    }

    enum { begin, preamble, frame, end, error } state = begin;
    state = begin;
    static uint8_t frame_data[1024];
    uint32_t frame_len = 0;
    int bitcnt = 0;
    int bitcnt2 = 0;
    uint8_t d2 = 0;
    for (int sample = 0; sample < n_samples && state != end; sample+=8) {
      uint32_t d = buf[sample/8];
      for (int i = 0; i < 8; i++) {
        if (!(d & 0x4)) {
          if (state < frame && (d & 0x7) == 0x0) {
            // DV=0 in or before preamble is ok-ish
            state = begin;
          } else {
            state = end;
            printf("eof\r\n");
            break;
          }
        }
        switch (state) {
          case begin:
            if ((d & 0x3) == 0)
              ;
            else if ((d & 0x3) == 1)
              state = preamble;
            else
              state = error;
            break;
          case preamble:
            if ((d & 0x3) == 1)
              ;
            else if ((d & 0x3) == 3)
              state = frame;
            else
              state = error;
            break;
          case frame:
            d2 |= (d & 0x3) << bitcnt;
            bitcnt += 2;
            bitcnt2 += 2;
            if (bitcnt == 8) {
              frame_data[frame_len++] = d2;
              bitcnt = 0;
              d2 = 0;
            }
            break;
          case end:
          case error:
            printf("state=%d\r\n", state);
            break;
        }
        //printf("%d: %1lx -> %d\r\n", sample+i, d&0xf, state);
        //if (state == frame && bitcnt2 > 0 && bitcnt == 0)
        //  printf("  byte %ld: %08x\r\n", frame_len-1, frame_data[frame_len-1]);
        d >>= 4;
      }
    }

    if (state == error)
      printf("state=error\r\n");
    printf("frame, len=%lu + %d bits, %d bits: ", frame_len, bitcnt, bitcnt2);
    for (int i=0; i<frame_len; i++) {
      if ((i % 16) == 0)
        printf("\r\n");
      printf(" %02x", frame_data[i]);
    }
    printf("\r\n");

    cm_t cm;
    p_cm_t p_cm = &cm;
    memset(p_cm, 0, sizeof(cm));

    p_cm->cm_width  = 32;
    p_cm->cm_poly   = 0x04C11DB7;
    p_cm->cm_init   = 0xFFFFFFFF;
    p_cm->cm_refin  = 1;
    p_cm->cm_refot  = 1;
    p_cm->cm_xorot  = 0xFFFFFFFF;

    cm_ini(p_cm);

    uint8_t j;
    for (j = 0; j < frame_len-4 && j < frame_len; j += 1) {
      cm_nxt(p_cm, frame_data[j]);
    }

    uint32_t crc = cm_crc(p_cm) & 0xffffffff;
    uint32_t frame_fcs = (frame_data[frame_len - 1] << 24) |
                         (frame_data[frame_len - 2] << 16) |
                         (frame_data[frame_len - 3] << 8) |
                          frame_data[frame_len - 4];

    printf("Calculated CRC: 0x%x, Frame FCS: 0x%x\n", crc, frame_fcs);
    (crc == frame_fcs) ? printf("Matched!\n") : printf("Not matched!\n");

    struct _frame {
      uint8_t dmac[6], smac[6];
      uint16_t ether_type;
      uint8_t version_ihl;
      uint8_t dscp_ecn;
      uint16_t total_length;
      uint16_t identification;
      uint16_t flags_fragment;
      uint8_t ttl;
      uint8_t protocol;
      uint16_t header_checksum;
      uint32_t src, dst;
    } __attribute__((packed)) *ipframe = (struct _frame*)frame_data;

    //NOTE values are little-endian so swapped here
    if (crc == frame_fcs && frame_len >= 20 && ipframe->ether_type == 0x0008 && (ipframe->version_ihl&0xf0) == 0x40
        && !(ipframe->flags_fragment&0x4) && ipframe->protocol == 0x01 && ipframe->dst == 0xff012a0a) {
      printf("This is a ping.\r\n");
    }
}

void init_clock() {
    // select ROSC because we are going to reconfigure the PLL
    clock_configure(clk_sys,
      CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
      CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_ROSC_CLKSRC,
      8 * MHZ, 8 * MHZ);

    // Reconfigure PLL sys to 1500 / 5 / 2 = 150MHz
    //pll_init(pll_sys, 1, 1500 * MHZ, 6, 2);
    //pll_init(pll_sys, 1, 1500 * MHZ, 5, 2);
    //pll_init(pll_sys, 1, 1500 * MHZ, 3, 2);
    pll_init(pll_sys, 1, 1200 * MHZ, 3, 2);  float freq = 200 * MHZ;
    //pll_init(pll_sys, 1, 1200 * MHZ, 6, 2);  float freq = 100 * MHZ;

    // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    freq, freq);

    // tell SDK about changed frequency for clk_peri
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    freq, freq);
}

static const uint MDC = 10;
static const uint MDIO = 11;

void mdio_write(uint8_t phy, uint8_t reg, uint16_t data) {
  gpio_init(MDC);
  gpio_init(MDIO);
  gpio_set_dir(MDIO, GPIO_OUT);
  gpio_put(MDIO, 1);
  gpio_set_dir(MDC, GPIO_OUT);
  gpio_put(MDC, 0);

  for (int i=0; i<64; i++) {
    sleep_us(1);
    gpio_put(MDC, 1);
    sleep_us(2);
    gpio_put(MDC, 0);
    sleep_us(1);
  }

  uint32_t transfer = data | (2<<16) | ((reg&0x1f) << 18) | ((phy&0x1f) << 23) | (0x5 << 28);
  for (int i=0; i<32; i++) {
    gpio_put(MDIO, transfer & (1<<31) ? 1 : 0);
    transfer <<= 1;
    sleep_us(1);
    gpio_put(MDC, 1);
    sleep_us(2);
    gpio_put(MDC, 0);
    sleep_us(1);
  }
}

uint16_t mdio_read(uint8_t phy, uint8_t reg) {
  gpio_init(MDC);
  gpio_init(MDIO);
  gpio_set_dir(MDIO, GPIO_OUT);
  gpio_put(MDIO, 1);
  gpio_set_dir(MDC, GPIO_OUT);
  gpio_put(MDC, 0);
  sleep_us(4);

  for (int i=0; i<64; i++) {
    sleep_us(1);
    gpio_put(MDC, 1);
    sleep_us(2);
    gpio_put(MDC, 0);
    sleep_us(1);
  }

  uint32_t transfer = (2<<16) | ((reg&0x1f) << 18) | ((phy&0x1f) << 23) | (0x6 << 28);
  for (int i=0; i<14; i++) {
    gpio_put(MDIO, transfer & (1<<31) ? 1 : 0);
    transfer <<= 1;
    sleep_us(1);
    gpio_put(MDC, 1);
    sleep_us(2);
    gpio_put(MDC, 0);
    sleep_us(1);
  }

  gpio_set_dir(MDIO, GPIO_IN);
  gpio_pull_up(MDIO);
  gpio_set_input_enabled(MDIO, true);

  uint32_t data_in = 0;
  for (int i=0; i<18; i++) {
    sleep_us(1);
    gpio_put(MDC, 1);
    sleep_us(2);
    gpio_put(MDC, 0);
    data_in <<= 1;
    data_in |= gpio_get(MDIO) ? 1 : 0;
    sleep_us(1);
  }

  //FIXME why?
  data_in >>= 1;

  return (uint16_t)data_in;
}

int main() {
    init_clock();

    stdio_init_all();
    printf("PIO logic analyser example\n");

    int reneg = 0;
    while (1) {
      int found = -1;
      printf("scan:  ");
      for (int phy=0; phy<32; phy++) {
        uint16_t value = mdio_read(phy, 0);
        //printf("phy %2d, reg 0: %04x\r\n", phy, value);
        printf(" %04x", value);
        if (phy==15)
          printf("\r\n       ");
        if (value != 0xffff)
          found = phy;
      }
      printf("\r\n");

      if (found >= 0) {
        printf("phy %2d:", found);
        for (int reg=0; reg<32; reg++) {
          uint16_t value = mdio_read(found, reg);
          //printf("phy %2d, reg %2d: %04x\r\n", found, reg, value);
          printf(" %04x", value);
          if (reg==15)
            printf("\r\n       ");
        }
        printf("\r\n");

        if (reneg++ > 4) {
          reneg = 0;

          // see https://github.com/espressif/esp-idf/blob/526f682397a8cfb74698c601fd2c5b30e1433837/components/esp_eth/src/esp_eth_phy_lan8720.c

          // power on
          uint16_t bmcr = mdio_read(found, 0);
          bmcr &= ~(1<<11);  // disable power down
          mdio_write(found, 0, bmcr);

          while ((bmcr = mdio_read(found, 0)) & (1<<11))
            sleep_ms(10);

          // reset phy
          bmcr |= (1<<15);
          mdio_write(found, 0, bmcr);
          while ((bmcr = mdio_read(found, 0)) & (1<<15))
            sleep_ms(10);

          // check ID
          uint16_t id1 = mdio_read(found, 2);
          uint16_t id2 = mdio_read(found, 3);
          if (id1 != 0x0007 || (id2 & 0xfff0) != 0xc0f0)
            printf("not a LAN8720: %04x, %04x\r\n", id1, id2);

          // advertise 100 Mbit, full-duplex
          mdio_write(found, 4, 0x0181);

          // restart auto-negotiation
          printf("start autoneg\r\n");
          mdio_write(found, 0, 0x3300);

          uint16_t bmsr;
          uint16_t pscsr;
          while (1) {
            bmsr = mdio_read(found, 1);
            pscsr = mdio_read(found, 31);
            if ((bmsr & (1<<5)) && (pscsr & (1<<12))) {
              printf("autoneg done\r\n");
              break;
            }
          }

          if ((pscsr & 0x1c) != 0x18) {
            printf("link is not 100 Mbit full-duplex, pscsr=%04x!\r\n", pscsr);
          } else {
            break;
          }
        }
      }

      sleep_ms(500);
    }

    uint32_t capture_buf[(CAPTURE_PIN_COUNT * CAPTURE_N_SAMPLES + 31) / 32];

    PIO pio = pio0;
    uint sm = 0;
    uint dma_chan = 0;

    uint offset = pio_add_program(pio, &rmii_rx_program);
    rmii_rx_program_init(pio, sm, offset);

    volatile uint32_t* PIO0_FSTAT = (volatile uint32_t*)(PIO0_BASE + 0x04);
    volatile uint32_t* PIO0_FDEBUG = (volatile uint32_t*)(PIO0_BASE + 0x08);
    volatile uint32_t* PIO0_IRQ = (volatile uint32_t*)(PIO0_BASE + 0x30);
    volatile uint32_t* DMA0_TRANS_CNT = (volatile uint32_t*)(DMA_BASE + 0x40*dma_chan + 0x008);

    //printf("Arming trigger\n");
    while (1) {

      gpio_init(8);
      gpio_set_dir(8, GPIO_IN);
      while (!gpio_get(8))
        ;

      *PIO0_FDEBUG = 0xffffffff;
      rmii_rx_arm(pio, sm, dma_chan, capture_buf, //;
          (CAPTURE_PIN_COUNT * CAPTURE_N_SAMPLES + 31) / 32);

      //FIXME race condition for *very* short frames
      *PIO0_IRQ = 0x1;  // ack previous interrupt
 
      // wait for SM to signal end of frame
      while (!(*PIO0_IRQ & 0x1));

      // wait for FIFO empty or DMA done
      while (!(*PIO0_FSTAT & (1<<8)) && dma_channel_is_busy(dma_chan));

      uint32_t n_samples = CAPTURE_N_SAMPLES - *DMA0_TRANS_CNT*8;

      dma_channel_abort(dma_chan);

      printf("FDEBUG: %08x\r\n", *PIO0_FDEBUG);
  
      print_capture_buf(capture_buf, CAPTURE_PIN_COUNT, n_samples);
    }
}
