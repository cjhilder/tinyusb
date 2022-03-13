
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

void print_error_text(FRESULT e) {
  switch (e) {
    case 0:   return; //FR_OK = 0,				/* (0) Succeeded */
    case 1:   printf("FR_DISK_ERR,			/* (1) A hard error occurred in the low level disk I/O layer */"); break;
    case 2:   printf("FR_INT_ERR,			/* (2) Assertion failed */"); break;
    case 3:   printf("FR_NOT_READY,		/* (3) The physical drive cannot work */"); break;
    case 4:   printf("FR_NO_FILE,				/* (4) Could not find the file */"); break;
    case 5:   printf("FR_NO_PATH,				/* (5) Could not find the path */"); break;
    case 6:   printf("FR_INVALID_NAME,		/* (6) The path name format is invalid */"); break;
    case 7:   printf("FR_DENIED,				/* (7) Access denied due to prohibited access or directory full */"); break;
    case 8:   printf("FR_EXIST,				/* (8) Access denied due to prohibited access */"); break;
    case 9:   printf("FR_INVALID_OBJECT,		/* (9) The file/directory object is invalid */"); break;
    case 10:  printf("FR_WRITE_PROTECTED,		/* (10) The physical drive is write protected */"); break;
    case 11:  printf("FR_INVALID_DRIVE,		/* (11) The logical drive number is invalid */"); break;
    case 12:  printf("FR_NOT_ENABLED,			/* (12) The volume has no work area */"); break;
    case 13:  printf("FR_NO_FILESYSTEM,		/* (13) There is no valid FAT volume */"); break;
    case 14:  printf("FR_MKFS_ABORTED,		/* (14) The f_mkfs() aborted due to any problem */"); break;
    case 15:  printf("FR_TIMEOUT,				/* (15) Could not get a grant to access the volume within defined period */"); break;
    case 16:  printf("FR_LOCKED,				/* (16) The operation is rejected according to the file sharing policy */"); break;
    case 17:  printf("FR_NOT_ENOUGH_CORE,		/* (17) LFN working buffer could not be allocated */"); break;
    case 18:  printf("FR_TOO_MANY_OPEN_FILES,	/* (18) Number of open files > FF_FS_LOCK */"); break;
    case 19:  printf("FR_INVALID_PARAMETER	/* (19) Given parameter is invalid */"); break;
    default:  printf("Unrecognised error %d", e);
  }
  printf("\n");
}
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
        printf("Disk label = %s, root directory contains...\n",label);

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

        FATFS* ff;
        DWORD space;
        UINT  quantity;
        FRESULT result = f_getfree("", &space, &ff);
        char  buffer[] = "hello new file";
        printf("Get free space on disk (err=%d) in sectors %d\n", result, space);
        printf("Writing a file...\n");
        result = f_open(&file, "test-file.txt", FA_CREATE_ALWAYS | FA_WRITE);
        printf("opening test-file.txt (err=%d)\n", result);
        print_error_text(result);
        result = f_write(&file, buffer, sizeof(buffer), &quantity);
        printf("writing (err=%d) with %d bytes written\n", result, quantity);
        print_error_text(result);
        result =  f_close(&file);
        printf("closing (err=%d)\n", result);
        print_error_text(result);
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
