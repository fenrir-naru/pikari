#include <stdio.h>
#include <stdlib.h>
#include "c8051f990.h"

#include "main.h"
#include "f99x_uart0.h"
#include "f99x_rtc.h"
#include "f99x_cs0.h"

typedef enum {
  OSC_DEFAULT = 0x02, /* OSC_LOW_POWER / 8 */
  OSC_PRECISE = 0x00,
  OSC_LOW_POWER = 0x04,
} osc_t;

static osc_t sysclk_sel(osc_t osc);
static void port_init();

#define software_reset() {RSTSRC = 0x10;} // RSTSRC.4(SWRSF) = 1 causes software reset

void print_hex(u32 v){
  char buf[8], *c = buf + sizeof(buf);
  do{
    u8 v2 = v & 0xF;
    *(--c) = (v2 < 10) ? ('0' + v2) : ('A' + v2 - 10);
    v >>= 4;
  }while(v > 0);
  uart0_write(c, buf + sizeof(buf) - c);
}

///< @see 15.5 Important Note: address 0x0000 of external memory may be overwritten by an indeterminate value.
static __xdata __at(0x00) u8 dummy = 0;

/**
 * @return (u8) pmu0cf snapshot when wake up
 */
static u8 sleep_and_restore(){
  __bit ea = EA;
  osc_t osc = sysclk_sel(OSC_DEFAULT);
  u8 pmu0cf;

  EA = 0;

  //PMU0MD |= 0x20; // POR Supply Monitor Disable if VDD < 2.4
  //PMU0FL = 0x01; // CS0 wake up is available for suspend, not sleep
  PMU0CF = 0x8F; // sleep and enable all wake up sources

  sysclk_sel(osc);
  if((pmu0cf = PMU0CF) & 0x10){ // wake up source is -RST
    ///< @see 15.5, wait 15us
    u8 i = 0xFF;
    while(i-- > 0);
  }
  PMU0CF |= 0x20; // clear wake up flag

  EA = ea;

  return pmu0cf;
}

void main() {
#if defined(DEBUG)
  sysclk_sel(OSC_PRECISE); // Initialize oscillator
#endif
  port_init(); // Initialize crossbar and GPIO

#if defined(DEBUG)
  uart0_init();
#endif

  rtc_init();
  rtc_set_alarm(0x1000); // approximate 8Hz

  EA = 1; // Global Interrupt enable

  rtc_start();

  while (1) {
    static __xdata u32 elapsed = 0;
    u16 cap;

    cs0_turn_on();
    CS0MX = 0x08; // P1.0
    CS0INT = 0;
    CS0BUSY = 1;
    while(!CS0INT);
    {
      static __bit repeat = 0;
      if((cap = cs0_data()) > 0x2000){
        if(!repeat){
          repeat = 1;
          P0 ^= 0xC0; // toggle LED
        }
      }else{
        repeat = 0;
      }
    }
    cs0_turn_off();

#if defined(DEBUG)
    print_hex(elapsed++);
    uart0_write(" ", 1);
    print_hex(cap);
    uart0_write(" ", 1);
    while(uart0_tx_active()); // wait for UART TX completion
#endif

    sleep_and_restore();
  }
}

/**
 *
 * @return (u16) system clock frequency in KHz
 */
u16 sysclk_Khz(u8 clksel){
  u16 res;
  switch(clksel & 0x07){
    case 0: res = 24500; break;
    case 4: res = 20000; break;
    case 2: default: res = 2500;
  }
  return res >> ((clksel & 0x70) >> 4);
}

/**
 *
 * @return (osc_t) previous selection
 */
static osc_t sysclk_sel(osc_t osc){
  u8 clksel = 0x02; // Low Power Oscillator divided by 8 (=20MHz+/-10% / 8) (default)
  u8 previous = CLKSEL & 0x77;
  if(previous == (u8)osc){
    return osc;
  }
  switch(previous){
    case OSC_PRECISE: break;
    case OSC_LOW_POWER: break;
    default: previous = OSC_DEFAULT; // forcefully change to default
  }

  FLSCL &= ~0x40; // BYPASS => 0 (default)
  switch(osc){
    case OSC_PRECISE:
      REG0CN |= 0x10; // OSCBIAS = 1 @see Table 19.2 Note 2
      OSCICN |= 0x80; // turn on
      CLKSEL = 0x00; // Precise Oscillator (=24.5MHz)
      while((CLKSEL & 0x80) == 0); // slow => fast
      FLSCL |= 0x40; //
      break;
    case OSC_LOW_POWER:
      clksel = 0x04; // Low power Oscillator (=20MHz+/-10% / 8)
    case OSC_DEFAULT:
    default:
      while((CLKSEL & 0x80) == 0); // fast => slow, wait RDY @see Sec.19 top page
      CLKSEL = clksel;
      OSCICN &= ~0x80; // turn off
      REG0CN &= ~0x10; // OSCBIAS = 0
      break;
  }

  if(sysclk_Khz(clksel) > 14000){
    FLSCL |= 0x40; // BYPASS => 1
  }

  return (osc_t)previous;
}


static void port_init() {
  // P0
  // 0 => VREF, 1 => AGND, 2-3 => GPIO,
  // 4 => UART0_TX, 5 => UART0_RX, 6-7 => GPIO
  P0MDIN = 0xFC; // 0, 1 => analog
  P0MDOUT = 0xDC; // 0, 1, 5 => open-drain
  P0 = 0xFC;
  P0SKIP = 0x03;  // for VREF/AGND

  // P1
  // 0-7 => analog
  P1MDIN = 0x00; // analog
  //P1MDOUT = 0x00; // open-drain (default)
  //P1 = 0xFF; // (default)
  //P1SKIP = 0x00;  // (default)
  
  // P2 (P2.7 only)
  // 7 => C2D
  //P2MDOUT = 0x00; // 7 => open-drain (default)
  //P2 = 0x80; // (default)

  XBR0 = 0x01;  // UART0 enabled
  //XBR1 = 0x00; // (default)
  XBR2 = 0xC0;  // Enable crossbar & Disable weak pull-up
}

unsigned char _sdcc_external_startup(){
  PCA0MD &= ~0x40; ///< Disable Watchdog timer
  return 0;
}
