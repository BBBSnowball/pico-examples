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

const uint CAPTURE_PIN_BASE = 6;
const uint CAPTURE_PIN_COUNT = 4;
const uint CAPTURE_N_SAMPLES = 2048; //96;
const uint CAPTURE_PIN_TRIGGER = 9;

void logic_analyser_init(PIO pio, uint sm, uint pin_base, uint pin_count, float div) {
    // Load a program to capture n pins. This is just a single `in pins, n`
    // instruction with a wrap.
    uint16_t capture_prog_instr[] = {
      pio_encode_wait_gpio(true, 8),  //CRS
      pio_encode_wait_gpio(false, CAPTURE_PIN_TRIGGER),
      pio_encode_wait_gpio(true,  CAPTURE_PIN_TRIGGER),
      pio_encode_in(pio_pins, pin_count),
    };
    struct pio_program capture_prog = {
            .instructions = capture_prog_instr,
            .length = 4,
            .origin = -1
    };
    uint offset = pio_add_program(pio, &capture_prog);

    // Configure state machine to loop over this `in` instruction forever,
    // with autopush enabled.
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, pin_base);
    sm_config_set_wrap(&c, offset+3, offset+3);
    sm_config_set_clkdiv(&c, div);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c);
}

void logic_analyser_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words,
                        uint trigger_pin, bool trigger_level) {
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

    //pio_sm_exec(pio, sm, pio_encode_wait_gpio(trigger_level, trigger_pin));
    pio_sm_set_enabled(pio, sm, true);
}

void print_capture_buf(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples) {
    // Display the capture buffer in text form, like this:
    // 00: __--__--__--__--__--__--
    // 01: ____----____----____----
    static int cnt = 0;
    printf("Capture %d:\n", cnt++);
    for (int pin = 0; pin < pin_count; ++pin) {
        printf("%02d: ", pin + pin_base);
        for (int sample = 0; sample < n_samples; ++sample) {
            uint bit_index = pin + sample * pin_count;
            bool level = !!(buf[bit_index / 32] & 1u << (bit_index % 32));
            printf(level ? "-" : "_");
        }
        printf("\n");
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

    logic_analyser_init(pio, sm, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, 1.f);

    //printf("Arming trigger\n");
    while (1) {

      gpio_init(8);
      gpio_set_dir(8, GPIO_IN);
      while (!gpio_get(8))
        ;

    logic_analyser_arm(pio, sm, dma_chan, capture_buf, //;
        (CAPTURE_PIN_COUNT * CAPTURE_N_SAMPLES + 31) / 32,
        CAPTURE_PIN_TRIGGER, true);

    //printf("Starting PWM example\n");
    // PWM example: -----------------------------------------------------------
    //gpio_set_function(CAPTURE_PIN_BASE, GPIO_FUNC_PWM);
    //gpio_set_function(CAPTURE_PIN_BASE + 1, GPIO_FUNC_PWM);
    //// Topmost value of 3: count from 0 to 3 and then wrap, so period is 4 cycles
    //pwm_hw->slice[0].top = 3;
    //// Divide frequency by two to slow things down a little
    //pwm_hw->slice[0].div = 4 << PWM_CH0_DIV_INT_LSB;
    //// Set channel A to be high for 1 cycle each period (duty cycle 1/4) and
    //// channel B for 3 cycles (duty cycle 3/4)
    //pwm_hw->slice[0].cc =
    //        (1 << PWM_CH0_CC_A_LSB) |
    //        (3 << PWM_CH0_CC_B_LSB);
    //// Enable this PWM slice
    //pwm_hw->slice[0].csr = PWM_CH0_CSR_EN_BITS;
    //// ------------------------------------------------------------------------

    dma_channel_wait_for_finish_blocking(dma_chan);

    print_capture_buf(capture_buf, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CAPTURE_N_SAMPLES);

    sleep_ms(300);
    }
}
