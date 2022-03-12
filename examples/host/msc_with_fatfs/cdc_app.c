//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
#include "tusb.h"
#include "FATFs/diskio.h"

typedef enum {
    USB_no_device,
    USB_connecting,
    USB_device_error,
    USB_not_memorystick,
    USB_good_to_go
} USB_state_t;
extern volatile USB_state_t     USB_state;

#if CFG_TUH_CDC

CFG_TUSB_MEM_SECTION static char serial_in_buffer[64] = { 0 };

void tuh_mount_cb(uint8_t dev_addr)
{
  printf("A %s device with address %d is mounted \r\n", (tuh_cdc_serial_is_mounted(dev_addr)? "serial" : ""), dev_addr);
  if (tuh_cdc_serial_is_mounted(dev_addr)) {
      USB_state = USB_not_memorystick;
  }
}

void tuh_umount_cb(uint8_t dev_addr)
{
  // application tear-down
  USB_state = USB_no_device;
  printf("A device with address %d is unmounted \r\n", dev_addr);
}

/* The code below this line is not used in the minimal MSC example */

// invoked ISR context
void tuh_cdc_xfer_isr(uint8_t dev_addr, xfer_result_t event, cdc_pipeid_t pipe_id, uint32_t xferred_bytes)
{
  (void) event;
  (void) pipe_id;
  (void) xferred_bytes;

  printf(serial_in_buffer);
  tu_memclr(serial_in_buffer, sizeof(serial_in_buffer));

  tuh_cdc_receive(dev_addr, serial_in_buffer, sizeof(serial_in_buffer), true); // waiting for next data
}

void cdc_task(void)
{

}

#endif
