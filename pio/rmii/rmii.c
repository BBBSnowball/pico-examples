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
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"

// Some logic to analyse:
#include "hardware/structs/pwm.h"

#include "rmii.pio.h"

#include "lwip/def.h"
#include "netif/ethernet.h"
#include "lwip/src/include/lwip/prot/ip4.h"
#include "lwip/inet_chksum.h"
#include "lwip/icmp.h"
#include "lwip/src/include/lwip/prot/etharp.h"

#include "zlib/zlib.h"

const uint CAPTURE_PIN_COUNT = 4;
const uint CAPTURE_N_SAMPLES = 2048; //96;

//#define CLK 9
#define TX_EN 5
#define TX0 3
#define TX1 (TX0 + 1)

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


#define TXMON_ENABLE 0
#define TXMON_PIN_BASE TX0
#define TXMON_PIN_COUNT 8 // must include all TX pins and CLK, i.e. 3 to 9, must probably be a power of 2
#define TXMON_N_SAMPLES 2048
#define TXMON_WAIT 0

void txmon_init(PIO pio, uint sm) {
    uint pin_base = TXMON_PIN_BASE;
    uint pin_count = TXMON_PIN_COUNT;

    // Load a program to capture n pins. This is just a single `in pins, n`
    // instruction with a wrap.
    uint16_t capture_prog_instr[] = {
#if TXMON_WAIT
      pio_encode_wait_gpio(true, TX_EN),
      pio_encode_wait_gpio(false, CLK),
      pio_encode_wait_gpio(true,  CLK),
#endif
      pio_encode_in(pio_pins, pin_count),
    };
    struct pio_program capture_prog = {
            .instructions = capture_prog_instr,
            .length = TXMON_WAIT ? 4 : 1,
            .origin = -1
    };
    uint offset = pio_add_program(pio, &capture_prog);
    printf("txmon program is at %d\r\n", offset);

    // Configure state machine to loop over this `in` instruction forever,
    // with autopush enabled.
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, pin_base);
    sm_config_set_wrap(&c, offset+capture_prog.length-1, offset+capture_prog.length-1);
    sm_config_set_clkdiv(&c, 1);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c);
}

void txmon_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words) {
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

    //pio_sm_set_enabled(pio, sm, true);
}

bool calc_fcs(uint8_t* frame_data, size_t frame_len, bool update);

void txmon_print_capture_buf(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples) {
    // Display the capture buffer in text form, like this:
    // 00: __--__--__--__--__--__--
    // 01: ____----____----____----
    static int cnt = 0;
    printf("Capture %d:\n", cnt++);
    for (int sample_start = 0; sample_start < n_samples; sample_start += 96) {
       if (sample_start > 0) printf("%d:\n", sample_start);
       for (int pin = 0; pin < pin_count; ++pin) {
           printf("%02d: ", pin + pin_base);
           for (int sample = sample_start; sample < n_samples && sample < sample_start + 96; ++sample) {
               uint bit_index = pin + sample * pin_count;
               bool level = !!(buf[bit_index / 32] & 1u << (bit_index % 32));
               printf(level ? "-" : "_");
           }
           printf("\n");
       }
    }


    enum { begin, preamble, frame, end, error } state = begin;
    state = begin;
    static uint8_t frame_data[1024];
    uint32_t frame_len = 0;
    int bitcnt = 0;
    int bitcnt2 = 0;
    uint8_t d2 = 0;
    uint8_t d = 0xff;
    for (int sample = 0; sample < n_samples && state != end && state != error; sample+=4) {
      uint32_t dnext = buf[sample/4];
      for (int i = 0; i < 4; i++) {
        if ((dnext & (1<<6)) && !(d & (1<<6))) {
          // rising edge of clock
        } else {
          d = dnext;
          dnext >>= 8;
          continue;
        }
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
        printf("%d: %1x -> %d\r\n", sample+i, d&0xf, state);
        if (state == frame && bitcnt2 > 0 && bitcnt == 0)
          printf("  byte %ld: %08x\r\n", frame_len-1, frame_data[frame_len-1]);

        d = dnext;
        dnext >>= 8;
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

    calc_fcs(frame_data, frame_len, false);
}


// Frame starts with length and preamble
#define tx_frame_preface_length ((32 + 12*8)/8)

// tx_buf must contain tx_frame_preface_length+frame_length bytes, rounded up to whole uint32_t
void rmii_tx_init_buf(uint32_t* tx_buf, size_t frame_length) {
  // transaction count: four clocks per octet
  //NOTE minus 4 because tx_frame_preface_length includes the length word
  tx_buf[0] = (frame_length + tx_frame_preface_length - 4) * 4;
  // preamble and SFD
  tx_buf[1] = 0x55555500;
  tx_buf[2] = 0x55555555;
  tx_buf[3] = 0xd5555555;
  // clear last word - not strictly necessary as the bits after the frame
  // shouldn't be used by the PIO
  tx_buf[frame_length/4] = 0;
}

static inline bool pio_sm_is_enabled(PIO pio, uint sm) {
  return !!(pio->ctrl & (1<<sm));
}

bool rmii_tx_can_send(PIO pio, uint sm, uint dma_chan) {
  //FIXME replace by pio->xx
  intptr_t PIO_BASE = pio == pio0 ? PIO0_BASE : PIO1_BASE;
  volatile uint32_t* PIO_IRQ = (volatile uint32_t*)(PIO_BASE + 0x30);

  if (!pio_sm_is_enabled(pio, sm)) {
    return true;
  } else {
    if (!(*PIO_IRQ & 0x2))
      // IRQ1 is not asserted -> PIO is still busy
      return false;
  }
}

bool rmii_tx_send(PIO pio, uint sm, uint dma_chan, uint32_t* tx_buf) {
  intptr_t PIO_BASE = pio == pio0 ? PIO0_BASE : PIO1_BASE;
  volatile uint32_t* PIO_IRQ = (volatile uint32_t*)(PIO_BASE + 0x30);

  static uint offset;

  if (!pio_sm_is_enabled(pio, sm)) {
    //FIXME move init code out of here
    offset = pio_add_program(pio, &rmii_tx_program);
    rmii_tx_program_init(pio, sm, offset, CLK, TX_EN, TX0);
    printf("tx program is at %d\r\n", offset);
  
    pio_sm_set_enabled(pio, sm, false);
  } else {
    if (!(*PIO_IRQ & 0x2))
      // IRQ1 is not asserted -> PIO is still busy
      return false;
  }

  printf("DBG_PADOUT: %08lx\r\n", pio->dbg_padout);
  pio_sm_set_enabled(pio, sm, false);
  pio_sm_clear_fifos(pio, sm);
  printf("DBG_PADOUT: %08lx\r\n", pio->dbg_padout);
  pio_sm_exec(pio, sm, pio_encode_jmp(offset));
  printf("DBG_PADOUT: %08lx\r\n", pio->dbg_padout);

  printf("jmp: %08x, %08x\r\n", pio_encode_jmp(offset), rmii_tx_program.instructions[rmii_tx_program.length-1]);

  // ack previous irq
  *PIO_IRQ = 0x2;

  dma_channel_config c = dma_channel_get_default_config(dma_chan);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

  dma_channel_configure(dma_chan, &c,
      &pio->txf[sm],      // Destinatinon pointer
      tx_buf,             // Source pointer
      (tx_buf[0]+15)/16+1,// Number of transfers
      true                // Start immediately
  );

  printf("number of dma transfers: %lu+1 for %lu clocks plus cnt\r\n", (tx_buf[0]+15)/16, tx_buf[0]);

  printf("DBG_PADOUT: %08lx\r\n", pio->dbg_padout);

  //pio_sm_set_enabled(pio, sm, true);

  return true;
}

bool calc_fcs(uint8_t* frame_data, size_t frame_len, bool update) {
    uint32_t good_fcs = crc32(0, frame_data, frame_len-4);

    //NOTE This is not swapping byte order, on purpose (because of the way the CRC is calculated).
    //     This will only work for little-endian, I think.
    uint32_t frame_fcs;
    memcpy(&frame_fcs, frame_data + frame_len - 4, 4);

    bool valid = (good_fcs == frame_fcs);

    if (!update) {
        printf("FCS: expected 0x%lx, actual 0x%lx -> %s\n", good_fcs, frame_fcs, valid ? "ok" : "wrong");
    } else {
        // dito not swapped, see above
        memcpy(frame_data + frame_len - 4, &good_fcs, 4);
    }

    return valid;
}

bool rmii_tx_can_send2();
bool rmii_tx_send2(uint32_t* tx_buf);

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
    for (int sample = 0; sample < n_samples && state != end && state != error; sample+=8) {
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

    bool fcs_valid = calc_fcs(frame_data, frame_len, false);

    struct _frame {
      struct eth_hdr eth;
      union {
        struct {
          struct ip_hdr ipv4;
          struct icmp_echo_hdr icmp_echo;
        };
        struct etharp_hdr arp;
      };
    } __attribute__((packed)) *rxframe = (struct _frame*)(frame_data - ETH_PAD_SIZE);

    uint8_t our_mac[6] = { 0x02, 0x43, 0x32, 0xb1, 0x67, 0xa7 }; // locally administered, random
    uint32_t our_ip = 0x05012a0a; // 10.42.1.5

    //NOTE This is recognizing very specific pings. They are generated like this:
    //     ip a add 10.42.1.1/24 dev ens10f0u1u4
    //     ping 10.42.1.255 -bf -I ens10f0u1u4 -s 10  // size can be different
    static uint32_t ok_cnt = 0, err_cnt = 0, ping_cnt = 0, tx_drop = 0;
    static uint8_t tx_data[1024 + tx_frame_preface_length + 4];
    //NOTE values are little-endian so swapped here
    if (fcs_valid && frame_len >= 20 && rxframe->eth.type == 0x0008 && IPH_V(&rxframe->ipv4) == 0x4
        && !(rxframe->ipv4._offset&0x4) && IPH_PROTO(&rxframe->ipv4) == 0x01 && (rxframe->ipv4.dest.addr == 0xff012a0a || rxframe->ipv4.dest.addr == our_ip)
        && rxframe->icmp_echo.type == ICMP_ECHO && rxframe->icmp_echo.code == 0) {
      printf("This is a ping.\r\n");
      ok_cnt++;
      ping_cnt++;

      if (!rmii_tx_can_send2()) {
        tx_drop++;
      } else {
        //FIXME I think the conversion from uint8_t to uint32_t is only ok for little-endian. Should we check or change that?
        rmii_tx_init_buf((uint32_t*)tx_data, frame_len);
        struct _frame* tx = (struct _frame*)(tx_data + tx_frame_preface_length);
        memcpy(tx, frame_data, frame_len);
        memcpy(&tx->eth.dest, &tx->eth.src, 6);
        //memcpy(&tx->eth.src, our_mac, 6);
        memcpy(&tx->eth.src, &rxframe->eth.dest, 6);
        tx->ipv4.dest = tx->ipv4.src;
        tx->ipv4.src = rxframe->ipv4.dest; //our_ip
        tx->icmp_echo.type = ICMP_ER; // echo reply

        struct ip_hdr* iphdr = (struct ip_hdr*)&tx->ipv4;
        IPH_CHKSUM_SET(iphdr, 0);
        IPH_CHKSUM_SET(iphdr, inet_chksum(iphdr, 20));

        struct icmp_echo_hdr* icmphdr = (struct icmp_echo_hdr *)&tx->icmp_echo;
        icmphdr->chksum = 0;
        icmphdr->chksum = inet_chksum(icmphdr, frame_len - ((uint8_t*)icmphdr - (uint8_t*)tx) - 4);

        calc_fcs(tx_data + tx_frame_preface_length, frame_len, true);
        calc_fcs(tx_data + tx_frame_preface_length, frame_len, false);

        // ethtool -K ens10f0u1u4 rx off tx off
        // tcpdump -i ens10f0u1u4 --direction=in
        // ethtool --statistics ens10f0u1u4
        // ethtool --phy-statistics ens10f0u1u4
        if (!rmii_tx_send2((uint32_t*)tx_data)) {
          tx_drop++;
        }
      }
    } else if (fcs_valid && frame_len >= 20 && rxframe->eth.type == htons(ETHTYPE_ARP) && rxframe->arp.hwtype == htons(0x0001)
        && rxframe->arp.proto == htons(0x0800) && rxframe->arp.hwlen == 6 && rxframe->arp.protolen == 4 && rxframe->arp.opcode == htons(1)
        && *(uint32_t*)&rxframe->arp.dipaddr == our_ip) {
      printf("This is an ARP request.\r\n");
      ok_cnt++;

      if (!rmii_tx_can_send2()) {
        tx_drop++;
      } else {
        //FIXME I think the conversion from uint8_t to uint32_t is only ok for little-endian. Should we check or change that?
        rmii_tx_init_buf((uint32_t*)tx_data, frame_len);
        struct _frame* tx = (struct _frame*)(tx_data + tx_frame_preface_length);
        memcpy(tx, frame_data, frame_len);
        memcpy(&tx->eth.dest, &tx->eth.src, 6);
        memcpy(&tx->eth.src, our_mac, 6);

        tx->arp.opcode = htons(2);
        memcpy(&tx->arp.dhwaddr, &rxframe->arp.shwaddr, 10);
        memcpy(&tx->arp.shwaddr, our_mac, 6);
        *(uint32_t*)&tx->arp.sipaddr = our_ip;

        calc_fcs(tx_data + tx_frame_preface_length, frame_len, true);

        if (!rmii_tx_send2((uint32_t*)tx_data)) {
          tx_drop++;
        }
      }
    } else if (fcs_valid) {
      ok_cnt++;
    } else {
      err_cnt++;
    }
    printf("ok=%lu, err=%lu, pings=%lu, tx_drops=%lu\r\n", ok_cnt, err_cnt, ping_cnt, tx_drop);
}

bool rmii_tx_can_send2() {
      PIO pio = pio0;
      uint sm = 1;
      uint dma_chan = 1;

      return rmii_tx_can_send(pio, sm, dma_chan);
}

bool rmii_tx_send2(uint32_t* tx_buf) {
      PIO pio = pio0;
      uint sm = 1;
      uint dma_chan = 1;

      PIO mon_pio = pio0;
      uint mon_sm = 2;
      uint mon_dma_chan = 2;

#if TXMON_ENABLE
        static uint32_t mon_buf[(TXMON_PIN_COUNT * TXMON_N_SAMPLES + 31) / 32];
        static bool txmon_initialized = false;
        if (!txmon_initialized) {
          txmon_initialized = true;
          txmon_init(mon_pio, mon_sm);
        }
        txmon_arm(mon_pio, mon_sm, mon_dma_chan, mon_buf, sizeof(mon_buf)/sizeof(*mon_buf));
#endif

        if (!rmii_tx_send(pio, sm, dma_chan, tx_buf)) {
          dma_channel_abort(mon_dma_chan);
          pio_sm_set_enabled(mon_pio, mon_sm, false);
          return false;
        } else {
          pio_sm_set_set_pins(pio, sm, 3, 2);

          if (0) {
            dma_channel_abort(dma_chan);
            printf("tx sm: out_base=%ld, out_count=%ld\r\n",
                (pio->sm[sm].pinctrl & PIO_SM0_PINCTRL_OUT_BASE_BITS) >> PIO_SM0_PINCTRL_OUT_BASE_LSB,
                (pio->sm[sm].pinctrl & PIO_SM0_PINCTRL_OUT_COUNT_BITS) >> PIO_SM0_PINCTRL_OUT_COUNT_LSB);

            // sanity test: can the txmon see pin changes that we do in the tx sm?
            printf("DBG_PADOE:  %08lx\r\n", pio->dbg_padoe);
            printf("DBG_PADOUT: %08lx\r\n", pio->dbg_padout);
            pio_sm_exec(mon_pio, mon_sm, pio_encode_set(pio_x, 0x05));
            pio_sm_exec(mon_pio, mon_sm, pio_encode_in(pio_x, 32));
            printf("txmon: x=5: %08lx\r\n", pio->rxf[mon_sm]);
            pio_sm_exec(mon_pio, mon_sm, pio_encode_in(pio_pins, 32));
            printf("txmon: pins=%08lx\r\n", pio->rxf[mon_sm]);

            for (int j=0; j<32; j++) {
              pio_sm_exec(pio, sm, pio_encode_set(pio_pins, j));
              //pio->txf[sm] = j;
              //pio_sm_exec(pio, sm, pio_encode_out(pio_pins, 3));
              pio_sm_exec(mon_pio, mon_sm, pio_encode_in(pio_pins, 32));
              printf("txmon: pins=%08lx (set by tx: %02x), DBG_PADOUT=%08lx\r\n", pio->rxf[mon_sm], j, pio->dbg_padout);
            }
            pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 0));
            pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 1));
            printf("DBG_PADOUT: %08lx\r\n", pio->dbg_padout);

            //while (1);
          }

          if (0) {
            const uint8_t* tx_data = (const char*)tx_buf;
            printf("tx buf:");
            for (int j=0; j<32; j++) {
              printf(" %02x", tx_data[j]);
            }
            printf(" ...\r\n");
          }

#if TXMON_ENABLE
          // pio_sm_set_enabled for tx and txmon statemachines at the same time
          memset(mon_buf, 0, sizeof(mon_buf));
          txmon_arm(mon_pio, mon_sm, mon_dma_chan, mon_buf, sizeof(mon_buf)/sizeof(*mon_buf));
          pio->ctrl |= (1<<sm) | (1<<mon_sm);
          printf("waiting for tx_mon...\r\n");
          dma_channel_wait_for_finish_blocking(mon_dma_chan);
          txmon_print_capture_buf(mon_buf, TXMON_PIN_BASE, TXMON_PIN_COUNT, TXMON_N_SAMPLES);
#else
          pio_sm_set_enabled(pio, sm, true);
#endif

          //while (1);
 
          return true;
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
    pll_init(pll_sys, 1, 1500 * MHZ, 3, 2);  float freq = 250 * MHZ;
    //pll_init(pll_sys, 1, 1200 * MHZ, 3, 2);  float freq = 200 * MHZ;
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
    printf("PIO RMII example\n");

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
    printf("rx program is at %d\r\n", offset);

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

      printf("FDEBUG: %08lx\r\n", *PIO0_FDEBUG);
  
      print_capture_buf(capture_buf, CAPTURE_PIN_COUNT, n_samples);
    }
}
