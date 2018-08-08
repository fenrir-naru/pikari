#include "f99x_rtc.h"

static __xdata u8 rtc0adr = 0x10; // short strobe
static __xdata u8 rtc0cn = 0x00;

void rtc_init(){
  rtc_write(RTC_RTC0XCN, 0x08); // LFO is selected
  rtc_write(RTC_RTC0CN, (rtc0cn |= 0x84)); // RTC, and auto reset enabled
}

void rtc_write(rtc_addr_t addr, u8 data){
  RTC0ADR = rtc0adr | (u8)addr;
  RTC0DAT = data;
}

u8 rtc_read(rtc_addr_t addr){
  RTC0ADR = 0xC0 | rtc0adr | (u8)addr; // auto read
  while((RTC0ADR & 0x80) != 0x00);
  return (u8)RTC0DAT;
}

void rtc_start(){
  rtc_write(RTC_RTC0CN, (rtc0cn |= 0x10)); // timer start
}

void rtc_stop(){
  rtc_write(RTC_RTC0CN, (rtc0cn &= ~0x10)); // timer stop
}

static void write_u32(rtc_addr_t top, u32 v){
  rtc_write(top, ((DWORD_t *)(&v))->c[0]);
  RTC0DAT = ((DWORD_t *)(&v))->c[1];
  RTC0DAT = ((DWORD_t *)(&v))->c[2];
  RTC0DAT = ((DWORD_t *)(&v))->c[3];
}

static u32 read_u32(rtc_addr_t top){
  DWORD_t res;
  res.c[0] = rtc_read(top);
  res.c[1] = RTC0DAT;
  res.c[2] = RTC0DAT;
  res.c[3] = RTC0DAT;
  return res.i;
}

void rtc_write_capture(u32 v) {
  write_u32(RTC_CAPTURE0, v);
  rtc_write(RTC_RTC0CN, rtc0cn | 0x02);
  while(rtc_read(RTC_RTC0CN) & 0x02);
}
u32 rtc_read_capture(){
  rtc_write(RTC_RTC0CN, rtc0cn | 0x01);
  while(rtc_read(RTC_RTC0CN) & 0x01);
  return read_u32(RTC_CAPTURE0);
}

void rtc_clear_alarm(){
  rtc_write(RTC_RTC0CN, (rtc0cn &= ~0x08));
}
void rtc_set_alarm(u32 v){
  rtc_write(RTC_RTC0CN, (rtc0cn & ~0x08));
  write_u32(RTC_ALARM0, v - 4); // auto reset and LFO are used
  rtc_write(RTC_RTC0CN, (rtc0cn |= 0x08));
}
