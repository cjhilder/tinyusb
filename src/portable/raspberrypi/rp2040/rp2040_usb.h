#ifndef RP2040_COMMON_H_
#define RP2040_COMMON_H_

#if defined(RP2040_USB_HOST_MODE) && defined(RP2040_USB_DEVICE_MODE)
#error TinyUSB device and host mode not supported at the same time
#endif

#include "common/tusb_common.h"

#include "pico.h"
#include "hardware/structs/usb.h"
#include "hardware/irq.h"
#include "hardware/resets.h"

#if defined(PICO_RP2040_USB_DEVICE_ENUMERATION_FIX) && !defined(TUD_OPT_RP2040_USB_DEVICE_ENUMERATION_FIX)
#define TUD_OPT_RP2040_USB_DEVICE_ENUMERATION_FIX PICO_RP2040_USB_DEVICE_ENUMERATION_FIX
#endif


#define pico_info(...)  TU_LOG(2, __VA_ARGS__)
#define pico_trace(...) TU_LOG(3, __VA_ARGS__)

typedef struct buffer_control_value {
    io_rw_16 value;
    uint8_t  index;
} buffer_control_value_t;

typedef enum {
    BUF_CTRL_OP_AND,
    BUF_CTRL_OP_OR,
    BUF_CTRL_OP_SET_MASK,
    BUF_CTRL_OP_CLR_MASK

} buf_ctrl_op_t;

// Hardware information per endpoint
typedef struct hw_endpoint
{
    // Is this a valid struct
    bool configured;
    
    // Transfer direction (i.e. IN is rx for host but tx for device)
    // allows us to common up transfer functions
    bool rx;
    
    uint8_t ep_addr;
    uint8_t next_pid;

    // Endpoint control register
    io_rw_32 *endpoint_control;

    // Buffer control register
    io_rw_16 *buffer_control;

    // Buffer pointer in usb dpram
    uint8_t *hw_data_buf;

    // Current transfer information
    bool active;
    uint16_t remaining_len;
    uint16_t xferred_len;

    // User buffer in main memory
    uint8_t *user_buf;

    // Data needed from EP descriptor
    uint16_t wMaxPacketSize;

    // Interrupt, bulk, etc
    uint8_t transfer_type;
    
#if CFG_TUH_ENABLED
    // Only needed for host
    uint8_t dev_addr;

    // If interrupt endpoint
    uint8_t interrupt_num;
#endif
} hw_endpoint_t;

void rp2040_usb_init(void);

void hw_endpoint_xfer_start(struct hw_endpoint *ep, uint8_t *buffer, uint16_t total_len);
bool hw_endpoint_xfer_continue(struct hw_endpoint *ep);
void hw_endpoint_reset_transfer(struct hw_endpoint *ep);

void _hw_endpoint_buffer_control_update16(struct hw_endpoint *ep, buf_ctrl_op_t op, buffer_control_value_t val);
static inline io_rw_32 _hw_endpoint_buffer_control_get_value32(struct hw_endpoint *ep) {
    return *ep->buffer_control;
}
static inline void _hw_endpoint_buffer_control_set_value16(struct hw_endpoint *ep, buffer_control_value_t value) {
    return _hw_endpoint_buffer_control_update16(ep, BUF_CTRL_OP_OR, value);
}
static inline void _hw_endpoint_buffer_control_set_mask16(struct hw_endpoint *ep, buffer_control_value_t value) {
    return _hw_endpoint_buffer_control_update16(ep, BUF_CTRL_OP_SET_MASK, value);
}
static inline void _hw_endpoint_buffer_control_clear_mask32(struct hw_endpoint *ep, buffer_control_value_t value) {
    return _hw_endpoint_buffer_control_update16(ep, BUF_CTRL_OP_CLR_MASK, value);
}

static inline uintptr_t hw_data_offset(uint8_t *buf)
{
    // Remove usb base from buffer pointer
    return (uintptr_t)buf ^ (uintptr_t)usb_dpram;
}

extern const char *ep_dir_string[];

#endif
