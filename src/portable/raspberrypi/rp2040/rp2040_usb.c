/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2021 Ha Thach (tinyusb.org) for Double Buffered
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUSB_MCU == OPT_MCU_RP2040

#include <stdlib.h>
#include "rp2040_usb.h"

// Direction strings for debug
const char *ep_dir_string[] = {
        "out",
        "in",
};

static inline void _hw_endpoint_lock_update(__unused struct hw_endpoint * ep, __unused int delta) {
    // todo add critsec as necessary to prevent issues between worker and IRQ...
    //  note that this is perhaps as simple as disabling IRQs because it would make
    //  sense to have worker and IRQ on same core, however I think using critsec is about equivalent.
}

static void _hw_endpoint_xfer_sync(struct hw_endpoint *ep);
static void _hw_endpoint_start_next_buffer(struct hw_endpoint *ep);

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

void rp2040_usb_init(void)
{
  // Reset usb controller
  reset_block(RESETS_RESET_USBCTRL_BITS);
  unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

  // Clear any previous state just in case
  // TODO Suppress warning array-bounds with gcc11
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
  memset(usb_hw, 0, sizeof(*usb_hw));
  memset(usb_dpram, 0, sizeof(*usb_dpram));
#pragma GCC diagnostic pop

  // Mux the controller to the onboard usb phy
  usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;
}

void hw_endpoint_reset_transfer(struct hw_endpoint *ep)
{
  ep->active = false;
  ep->remaining_len = 0;
  ep->xferred_len = 0;
  ep->user_buf = 0;
}

void _hw_endpoint_buffer_control_update16(struct hw_endpoint *ep, buf_ctrl_op_t op, buffer_control_value_t val) {
    io_rw_16 ctrl_value = 0;
    io_rw_16 and_mask = 0;
    io_rw_16 or_mask = 0;
    assert(val.index < 2);
    assert(ep);
    io_rw_16 current = ep->buffer_control[val.index];
    switch (op) {
      case BUF_CTRL_OP_AND:
        and_mask = val.value;
        break;
      case BUF_CTRL_OP_OR:
        or_mask = val.value;
        break;
      case BUF_CTRL_OP_SET_MASK:
        and_mask = ~val.value;
        or_mask = val.value;
        break;
      case BUF_CTRL_OP_CLR_MASK:
        and_mask = ~val.value;
        break;
    }
    if (and_mask) {
        ctrl_value = current & and_mask;
    }
    if (or_mask) {
        ctrl_value |= or_mask;
    }
    // We absolutely cannot ever *write* to the buffer control register
    // whilst it is accessible to the controller. Therefore we must abort
    // the write if AVAIL bit is set, or wait for it to be cleared by the controller. 
    // The option chosen here is abort.
    if (current & USB_BUF_CTRL_AVAIL) {
      panic("ep %d %s (buffer %d) was in use by controller", tu_edpt_number(ep->ep_addr), ep_dir_string[tu_edpt_dir(ep->ep_addr)]), val.index;
    }
    ep->buffer_control[val.index] = ctrl_value & ~USB_BUF_CTRL_AVAIL;
    asm volatile("nop \n nop \n nop");
    ep->buffer_control[val.index] = ctrl_value;
}

// prepare buffer, return buffer control
static buffer_control_value_t prepare_ep_buffer(struct hw_endpoint *ep, uint8_t buf_id)
{
  assert(ep);
  assert(buf_id < 2);

  uint16_t buflen = tu_min16(ep->remaining_len, ep->wMaxPacketSize);
  
  ep->remaining_len = (uint16_t)(ep->remaining_len - buflen);

  io_rw_16 buf_ctrl = buflen | USB_BUF_CTRL_AVAIL;

  // PID
  buf_ctrl |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
  ep->next_pid ^= 1u;

  if ( !ep->rx )
  {
    // transmit
    // Copy data from user buffer to hw buffer
    assert((ep->user_buf && buflen) || (buflen == 0));
    assert(buflen <= ep->wMaxPacketSize);
    assert((ep->wMaxPacketSize == 64) || (buflen == 0));
    if (buflen) {
      TU_LOG(2, "Memory copy 0x%x to 0x%x with len %d\n", ep->user_buf, ep->hw_data_buf + buf_id*64, buflen);
      memcpy(ep->hw_data_buf + buf_id*64, ep->user_buf, buflen);
    }
    ep->user_buf += buflen;

    // Mark as full
    buf_ctrl |= USB_BUF_CTRL_FULL;
  }

  // Is this the last buffer? Only really matters for host mode. Will trigger
  // the trans complete irq but also stop it polling. We only really care about
  // trans complete for setup packets being sent
  if (ep->remaining_len == 0)
  {
    buf_ctrl |= USB_BUF_CTRL_LAST;
  }

  buffer_control_value_t result;
  result.value = buf_ctrl;
  result.index = buf_id;
  return result;
}

// Prepare buffer control register value
static void _hw_endpoint_start_next_buffer(struct hw_endpoint *ep)
{
  assert(ep);
  assert(ep->endpoint_control);
  uint32_t ep_ctrl = *ep->endpoint_control;
  buffer_control_value_t buf_ctrl_0 = {.value = 0, .index = 0};
  buffer_control_value_t buf_ctrl_1 = {.value = 0, .index = 1};
  bool double_buffered = false;
  
  // always compute and start with buffer 0
  buf_ctrl_0 = prepare_ep_buffer(ep, 0);
  buf_ctrl_0.value |= USB_BUF_CTRL_SEL;

  // For now: skip double buffered for Device mode, OUT endpoint since
  // host could send < 64 bytes and cause short packet on buffer0
  // NOTE this could happen to Host mode IN endpoint
  bool const force_single = !(usb_hw->main_ctrl & USB_MAIN_CTRL_HOST_NDEVICE_BITS) && !tu_edpt_dir(ep->ep_addr);

  if(ep->remaining_len && !force_single)
  {
    // Use buffer 1 (double buffered) if there is still data
    // TODO: Isochronous for buffer1 bit-field is different than CBI (control bulk, interrupt)
    double_buffered = true;
    buf_ctrl_1 = prepare_ep_buffer(ep, 1);

    // Set endpoint control double buffered bit if needed
    ep_ctrl &= ~EP_CTRL_INTERRUPT_PER_BUFFER;
    ep_ctrl |= EP_CTRL_DOUBLE_BUFFERED_BITS | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER;
  }else
  {
    // Single buffered since 1 is enough
    ep_ctrl &= ~(EP_CTRL_DOUBLE_BUFFERED_BITS | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER);
    ep_ctrl |= EP_CTRL_INTERRUPT_PER_BUFFER;
  }

  *ep->endpoint_control = ep_ctrl;

  TU_LOG(2, "  Prepare BufCtrl: (double=%d) [buf0] = 0x%x  [buf1] = 0x%x\r\n", double_buffered, buf_ctrl_0.value, buf_ctrl_1.value);

  // Finally, write to buffer_control which will trigger the transfer
  // the next time the controller polls this dpram address
  assert(buf_ctrl_0.index == 0);
  assert(buf_ctrl_1.index == 1);
  _hw_endpoint_buffer_control_set_value16(ep, buf_ctrl_0);
  if (double_buffered) _hw_endpoint_buffer_control_set_value16(ep, buf_ctrl_1);
}

void hw_endpoint_xfer_start(struct hw_endpoint *ep, uint8_t *buffer, uint16_t total_len)
{
  assert(ep);
  assert(buffer || (total_len == 0));
  _hw_endpoint_lock_update(ep, 1);

  if ( ep->active )
  {
    // TODO: Is this acceptable for interrupt packets?
    TU_LOG(1, "WARN: starting new transfer on already active ep %d %s\n", tu_edpt_number(ep->ep_addr),
              ep_dir_string[tu_edpt_dir(ep->ep_addr)]);

    hw_endpoint_reset_transfer(ep);
  }

  // Fill in info now that we're kicking off the hw
  ep->remaining_len = total_len;
  ep->xferred_len   = 0;
  ep->active        = true;
  ep->user_buf      = buffer;

  _hw_endpoint_start_next_buffer(ep);
  _hw_endpoint_lock_update(ep, -1);
}

// sync endpoint buffer and return transferred bytes
static uint16_t sync_ep_buffer(struct hw_endpoint *ep, uint8_t buf_id)
{
  assert(ep);
  assert(buf_id < 2);

  volatile io_rw_16 buf_ctrl = _hw_endpoint_buffer_control_get_value16(ep, buf_id);

  uint16_t xferred_bytes = buf_ctrl & USB_BUF_CTRL_LEN_MASK;

  if (xferred_bytes == 0) return 0;

  if ( !ep->rx )
  {
    // We are continuing a transfer here. If we are TX, we have successfully
    // sent some data can increase the length we have sent
    assert(!(buf_ctrl & USB_BUF_CTRL_FULL));

    ep->xferred_len = (uint16_t)(ep->xferred_len + xferred_bytes);
  }else
  {
    // If we have received some data, so can increase the length
    // we have received AFTER we have copied it to the user buffer at the appropriate offset
    assert(buf_ctrl & USB_BUF_CTRL_FULL);
    assert(ep->user_buf);
    TU_LOG(2, "Memory copy 0x%x to 0x%x with len %d\n", ep->hw_data_buf + buf_id*64, ep->user_buf, xferred_bytes);
    memcpy(ep->user_buf, ep->hw_data_buf + buf_id*64, xferred_bytes);
    ep->xferred_len = (uint16_t)(ep->xferred_len + xferred_bytes);
    ep->user_buf += xferred_bytes;
  }

  // Short packet
  if (xferred_bytes < ep->wMaxPacketSize)
  {
    pico_trace("  Short packet on buffer %d with %u bytes\n", buf_id, xferred_bytes);
    // Reduce total length as this is last packet
    ep->remaining_len = 0;
  }

  return xferred_bytes;
}

static void _hw_endpoint_xfer_sync (struct hw_endpoint *ep)
{
  // Update hw endpoint struct with info from hardware
  // after a buff status interrupt
  assert(ep);
  assert(ep->endpoint_control);

  io_rw_16 __unused buf_ctrl_0 = _hw_endpoint_buffer_control_get_value16(ep, 0);
  io_rw_16 __unused buf_ctrl_1 = _hw_endpoint_buffer_control_get_value16(ep, 1);
  TU_LOG(3, "  Sync BufCtrl: [0] = 0x%x  [1] = 0x%x\r\n", buf_ctrl_0, buf_ctrl_1);

  // always sync buffer 0
  uint16_t buf0_bytes = sync_ep_buffer(ep, 0);

  // sync buffer 1 if double buffered
  if ( (*ep->endpoint_control) & EP_CTRL_DOUBLE_BUFFERED_BITS )
  {
    //if (buf0_bytes == ep->wMaxPacketSize)
    //{
      // sync buffer 1 if not short packet
      sync_ep_buffer(ep, 1);
    //}else
    //{
      // short packet on buffer 0
      // TODO couldn't figure out how to handle this case which happen with net_lwip_webserver example
      // At this time (currently trigger per 2 buffer), the buffer1 is probably filled with data from
      // the next transfer (not current one). For now we disable double buffered for device OUT
      // NOTE this could happen to Host IN
#if 0
      uint8_t const ep_num = tu_edpt_number(ep->ep_addr);
      uint8_t const dir =  (uint8_t) tu_edpt_dir(ep->ep_addr);
      uint8_t const ep_id = 2*ep_num + (dir ? 0 : 1);

      // abort queued transfer on buffer 1
      usb_hw->abort |= TU_BIT(ep_id);

      while ( !(usb_hw->abort_done & TU_BIT(ep_id)) ) {}

      uint32_t ep_ctrl = *ep->endpoint_control;
      ep_ctrl &= ~(EP_CTRL_DOUBLE_BUFFERED_BITS | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER);
      ep_ctrl |= EP_CTRL_INTERRUPT_PER_BUFFER;

      _hw_endpoint_buffer_control_set_value32(ep, 0);

      usb_hw->abort &= ~TU_BIT(ep_id);

      TU_LOG(3, "----SHORT PACKET buffer0 on EP %02X:\r\n", ep->ep_addr);
      TU_LOG(3, "  BufCtrl: [0] = 0x%04u  [1] = 0x%04x\r\n", tu_u32_low16(buf_ctrl), tu_u32_high16(buf_ctrl));
#endif
    //}
  }
}

// Returns true if transfer is complete
bool hw_endpoint_xfer_continue(struct hw_endpoint *ep)
{
  assert(ep);

  _hw_endpoint_lock_update(ep, 1);
  // Part way through a transfer
  if (!ep->active)
  {
    panic("Can't continue xfer on inactive ep %d %s", tu_edpt_number(ep->ep_addr), ep_dir_string[tu_edpt_dir(ep->ep_addr)]);
  }

  // Update EP struct from hardware state
  _hw_endpoint_xfer_sync(ep);

  // Now we have synced our state with the hardware. Is there more data to transfer?
  // If we are done then notify tinyusb
  if (ep->remaining_len == 0)
  {
    pico_trace("Completed transfer of %d bytes on ep %d %s\n",
               ep->xferred_len, tu_edpt_number(ep->ep_addr), ep_dir_string[tu_edpt_dir(ep->ep_addr)]);
    // Notify caller we are done so it can notify the tinyusb stack
    _hw_endpoint_lock_update(ep, -1);
    return true;
  }
  else
  {
    _hw_endpoint_start_next_buffer(ep);
  }

  _hw_endpoint_lock_update(ep, -1);
  // More work to do
  return false;
}

#endif
