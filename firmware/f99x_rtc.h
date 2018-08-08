#ifndef __F99X_RTC_H__
#define __F99X_RTC_H__

#include "c8051f990.h"
#include "type.h"

typedef enum {
  RTC_CAPTURE0  = 0x00,
  RTC_CAPTURE1  = 0x01,
  RTC_CAPTURE2  = 0x02,
  RTC_CAPTURE3  = 0x03,
  RTC_RTC0CN    = 0x04,
  RTC_RTC0XCN   = 0x05,
  RTC_RTC0XCF   = 0x06,
  RTC_ALARM0    = 0x08,
  RTC_ALARM1    = 0x09,
  RTC_ALARM2    = 0x0A,
  RTC_ALARM3    = 0x0B,
} rtc_addr_t;

void rtc_init();

void rtc_write(rtc_addr_t addr, u8 data);
u8 rtc_read(rtc_addr_t addr);

void rtc_start();
void rtc_stop();

void rtc_write_capture(u32 v);
u32 rtc_read_capture();
void rtc_set_alarm(u32 v);
void rtc_clear_alarm();

#endif /* __F99X_RTC_H__ */
