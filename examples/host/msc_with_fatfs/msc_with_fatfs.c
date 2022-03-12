
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "tusb.h"
#include "FATFs/diskio.h"

typedef enum {
    USB_no_device,
    USB_connecting,
    USB_device_error,
    USB_not_memorystick,
    USB_good_to_go
} USB_state_t;

volatile USB_state_t     USB_state;

FATFS                    DiskFATState;
FIL                      file;

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+
void led_blinking_task(void);

extern void cdc_task(void);
extern void hid_app_task(void);

/*------------- MAIN -------------*/
int main(void)
{
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  stdio_uart_init_full(uart1, 115200, 8, 9);

  printf("\fTinyUSB Host MSC with FatFs Example\n");

  tusb_init();

  int loopCount =0;
  while (1)
  {
    tuh_task();
    led_blinking_task();

    if (USB_state == USB_connecting)    {
      FRESULT result;
      printf("FatFs mounting\n");
      result = f_mount(&DiskFATState, "" , 1);
      printf("Result = %d\n", result);
      if (result==FR_OK) {
          USB_state = USB_good_to_go;
      } else {
          USB_state = USB_device_error;
      }
    }
    if (USB_state == USB_good_to_go) {
        char label[24];
        label[0] = 0;
        f_getlabel("", label, 0);
        printf("Disk label = %s\n",label);

        static FILINFO fileInfo;
        DIR dirInfo;

        FRESULT res = f_findfirst(&dirInfo, &fileInfo, "", "*.*");
        if (fileInfo.altname[0] == 0) {
          printf("%s\n",fileInfo.fname);
        } else {
          printf("%s aka %s\n",fileInfo.fname, fileInfo.altname);
        }
        while (1) {
          res = f_findnext(&dirInfo, &fileInfo);
          if (res != FR_OK || fileInfo.fname[0] == 0) {
            break;
          }
          if (!(fileInfo.fname[0] == '.' || fileInfo.fattrib & (AM_HID | AM_SYS))) {  // skip hidden or system files
            if (fileInfo.altname[0] == 0) {
              printf("%s\n",fileInfo.fname);
            } else {
              printf("%s aka %s\n",fileInfo.fname, fileInfo.altname);
            }
          }
        }
        busy_wait_ms(1000);
        printf("loop count %d \n\n\n", loopCount++);
    
    }
  }
  return 0;
}

void led_blinking_task(void)
{
  const uint64_t interval_us = 1000000;
  static uint64_t start_us = 0;

  static bool led_state = false;

  // Blink every interval ms
  if ( time_us_64() - start_us < interval_us) return; // not enough time
  start_us += interval_us;

  gpio_put(PICO_DEFAULT_LED_PIN, led_state);
  led_state = 1 - led_state; // toggle
}
