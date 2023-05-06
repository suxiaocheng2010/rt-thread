/*
 * Copyright (C) 2023-2023 BYD, All Rights Reserved.
 *
 * SPDX-License-Identifier: The MIT License (MIT)
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-20     suxiaocheng  Init version
 */

#include <rtthread.h>

#include <rtdevice.h>
#include <string.h>

#ifdef RT_USING_USB_DEVICE

#include <drv_usbd.h>

static rt_bool_t address_setup_seq = false;

static struct udcd pico_udc;
static struct ep_id pico_ep_pool[] = {
	{0x0, USB_EP_ATTR_CONTROL, USB_DIR_INOUT, 64, ID_ASSIGNED},
	{0x1, USB_EP_ATTR_BULK, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0x1, USB_EP_ATTR_BULK, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0x2, USB_EP_ATTR_BULK, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0x2, USB_EP_ATTR_BULK, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0x3, USB_EP_ATTR_BULK, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0x3, USB_EP_ATTR_BULK, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0x4, USB_EP_ATTR_INT, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0x4, USB_EP_ATTR_INT, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0x5, USB_EP_ATTR_INT, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0x5, USB_EP_ATTR_INT, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0x6, USB_EP_ATTR_INT, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0x6, USB_EP_ATTR_INT, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0x7, USB_EP_ATTR_BULK, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0x7, USB_EP_ATTR_BULK, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0x8, USB_EP_ATTR_BULK, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0x8, USB_EP_ATTR_BULK, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0x9, USB_EP_ATTR_BULK, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0x9, USB_EP_ATTR_BULK, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0xa, USB_EP_ATTR_BULK, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0xa, USB_EP_ATTR_BULK, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0xb, USB_EP_ATTR_BULK, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0xb, USB_EP_ATTR_BULK, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0xc, USB_EP_ATTR_BULK, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0xc, USB_EP_ATTR_BULK, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0xd, USB_EP_ATTR_BULK, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0xd, USB_EP_ATTR_BULK, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0xe, USB_EP_ATTR_BULK, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0xe, USB_EP_ATTR_BULK, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0xf, USB_EP_ATTR_BULK, USB_DIR_IN, 64, ID_UNASSIGNED},
	{0xf, USB_EP_ATTR_BULK, USB_DIR_OUT, 64, ID_UNASSIGNED},
	{0xFF, USB_EP_ATTR_TYPE_MASK, USB_DIR_MASK, 0, ID_ASSIGNED},
};

// Init these in dcd_init
static uint8_t *next_buffer_ptr;

// USB_MAX_ENDPOINTS Endpoints, direction TUSB_DIR_OUT for out and TUSB_DIR_IN for in.
static struct hw_endpoint hw_endpoints[USB_MAX_ENDPOINTS][2];

// SOF may be used by remote wakeup as RESUME, this indicate whether SOF is actually used by usbd
static bool _sof_enable = false;

// Direction strings for debug
const char *ep_dir_string[] = {
	"out",
	"in",
};

// if usb hardware is in host mode
TU_ATTR_ALWAYS_INLINE static inline bool is_host_mode(void)
{
	return (usb_hw->
		main_ctrl & USB_MAIN_CTRL_HOST_NDEVICE_BITS) ? true : false;
}

TU_ATTR_ALWAYS_INLINE static inline struct hw_endpoint
    *hw_endpoint_get_by_num(uint8_t num, tusb_dir_t dir)
{
	return &hw_endpoints[num][dir];
}

#if TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX
volatile uint32_t e15_last_sof = 0;

// check if Errata 15 is needed for this endpoint i.e device bulk-in
static bool FASE_IRQ e15_is_bulkin_ep(struct hw_endpoint *ep)
{
	return (!is_host_mode() && tu_edpt_dir(ep->ep_addr) == TUSB_DIR_IN &&
		ep->transfer_type == TUSB_XFER_BULK);
}

// check if we need to apply Errata 15 workaround : i.e
// Endpoint is BULK IN and is currently in critical frame period i.e 20% of last usb frame
static bool FASE_IRQ e15_is_critical_frame_period(struct hw_endpoint *ep)
{
	TU_VERIFY(e15_is_bulkin_ep(ep));

	/* Avoid the last 200us (uframe 6.5-7) of a frame, up to the EOF2 point.
	 * The device state machine cannot recover from receiving an incorrect PID
	 * when it is expecting an ACK.
	 */
	uint32_t delta = time_us_32() - e15_last_sof;
	if (delta < 800 || delta > 998) {
		return false;
	}
	TU_LOG(3, "Avoiding sof %lu now %lu last %lu\n",
	       (usb_hw->sof_rd + 1) & USB_SOF_RD_BITS, time_us_32(),
	       e15_last_sof);
	return true;
}
#else
#define e15_is_bulkin_ep(x)             (false)
#define e15_is_critical_frame_period(x) (false)
#endif

static void _hw_endpoint_alloc(struct hw_endpoint *ep, uint8_t transfer_type)
{
	// size must be multiple of 64
	uint size = tu_div_ceil(ep->wMaxPacketSize, 64) * 64u;

	// double buffered Bulk endpoint
	if (transfer_type == TUSB_XFER_BULK) {
		size *= 2u;
	}

	ep->hw_data_buf = next_buffer_ptr;
	next_buffer_ptr += size;

	assert(((uintptr_t) next_buffer_ptr & 0b111111u) == 0);
	uint dpram_offset = hw_data_offset(ep->hw_data_buf);
	hard_assert(hw_data_offset(next_buffer_ptr) <= USB_DPRAM_MAX);

	pico_trace_init("  Allocated %d bytes at offset 0x%x (0x%p)\r\n", size,
			dpram_offset, ep->hw_data_buf);

	// Fill in endpoint control register with buffer offset
	uint32_t const reg =
	    EP_CTRL_ENABLE_BITS | ((uint) transfer_type <<
				   EP_CTRL_BUFFER_TYPE_LSB) | dpram_offset;

	*ep->endpoint_control = reg;
}

static struct hw_endpoint FASE_IRQ *hw_endpoint_get_by_addr(uint8_t ep_addr)
{
	uint8_t num = tu_edpt_number(ep_addr);
	tusb_dir_t dir = tu_edpt_dir(ep_addr);
	return hw_endpoint_get_by_num(num, dir);
}

static void hw_endpoint_init(uint8_t ep_addr, uint16_t wMaxPacketSize,
			     uint8_t transfer_type)
{
	struct hw_endpoint *ep = hw_endpoint_get_by_addr(ep_addr);

	const uint8_t num = tu_edpt_number(ep_addr);
	const tusb_dir_t dir = tu_edpt_dir(ep_addr);

	pico_trace_init("%s: ep%x, type: %x\n", __func__, ep_addr,
			transfer_type);

	ep->ep_addr = ep_addr;

	// For device, IN is a tx transfer and OUT is an rx transfer
	ep->rx = (dir == TUSB_DIR_OUT);

	ep->next_pid = 0u;
	ep->wMaxPacketSize = wMaxPacketSize;
	ep->transfer_type = transfer_type;

	// Every endpoint has a buffer control register in dpram
	if (dir == TUSB_DIR_IN) {
		ep->buffer_control = &usb_dpram->ep_buf_ctrl[num].in;
	} else {
		ep->buffer_control = &usb_dpram->ep_buf_ctrl[num].out;
	}

	// Clear existing buffer control state
	*ep->buffer_control = 0;

	if (num == 0) {
		// EP0 has no endpoint control register because the buffer offsets are fixed
		ep->endpoint_control = NULL;

		// Buffer offset is fixed (also double buffered)
		ep->hw_data_buf = (uint8_t *) & usb_dpram->ep0_buf_a[0];
	} else {
		// Set the endpoint control register (starts at EP1, hence num-1)
		if (dir == TUSB_DIR_IN) {
			ep->endpoint_control = &usb_dpram->ep_ctrl[num - 1].in;
		} else {
			ep->endpoint_control = &usb_dpram->ep_ctrl[num - 1].out;
		}

		// alloc a buffer and fill in endpoint control register
		_hw_endpoint_alloc(ep, transfer_type);

		pico_trace_init("%s: %x%s ctrl: %x\n", __func__, num,
				dir == TUSB_DIR_IN ? "in" : "out",
				*(ep->endpoint_control));
	}
}

static void _hw_endpoint_close(struct hw_endpoint *ep)
{
	// Clear hardware registers and then zero the struct
	// Clears endpoint enable
	*ep->endpoint_control = 0;
	// Clears buffer available, etc
	*ep->buffer_control = 0;
	// Clear any endpoint state
	memset(ep, 0, sizeof(struct hw_endpoint));

	// Reclaim buffer space if all endpoints are closed
	bool reclaim_buffers = true;
	for (uint8_t i = 1; i < USB_MAX_ENDPOINTS; i++) {
		if (hw_endpoint_get_by_num(i, TUSB_DIR_OUT)->hw_data_buf != NULL
		    || hw_endpoint_get_by_num(i,
					      TUSB_DIR_IN)->hw_data_buf !=
		    NULL) {
			reclaim_buffers = false;
			break;
		}
	}
	if (reclaim_buffers) {
		next_buffer_ptr = &usb_dpram->epx_data[0];
	}
}

static void hw_endpoint_close(uint8_t ep_addr)
{
	struct hw_endpoint *ep = hw_endpoint_get_by_addr(ep_addr);
	_hw_endpoint_close(ep);
}

static void reset_non_control_endpoints(void)
{
	// Disable all non-control
	for (uint8_t i = 0; i < USB_MAX_ENDPOINTS - 1; i++) {
		usb_dpram->ep_ctrl[i].in = 0;
		usb_dpram->ep_ctrl[i].out = 0;
	}

	// clear non-control hw endpoints
	tu_memclr(hw_endpoints[1],
		  sizeof(hw_endpoints) - 2 * sizeof(hw_endpoint_t));

	// reclaim buffer space
	next_buffer_ptr = &usb_dpram->epx_data[0];
}

// disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(__unused uint8_t rhport)
{
	(void)rhport;
	usb_hw_clear->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

// connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(__unused uint8_t rhport)
{
	(void)rhport;
	usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

void dcd_int_enable(__unused uint8_t rhport)
{
	assert(rhport == 0);
	irq_set_enabled(USBCTRL_IRQ, true);
}

void dcd_int_disable(__unused uint8_t rhport)
{
	assert(rhport == 0);
	irq_set_enabled(USBCTRL_IRQ, false);
}

void FASE_IRQ _hw_endpoint_buffer_control_update32(struct hw_endpoint *ep,
					  uint32_t and_mask, uint32_t or_mask)
{
	uint32_t value = 0;

	if (and_mask) {
		value = *ep->buffer_control & and_mask;
	}

	if (or_mask) {
		value |= or_mask;
		if (or_mask & USB_BUF_CTRL_AVAIL) {
			if (*ep->buffer_control & USB_BUF_CTRL_AVAIL) {
				// panic("ep %d %s was already available", tu_edpt_number(ep->ep_addr), ep_dir_string[tu_edpt_dir(ep->ep_addr)]);
				pico_trace
				    ("Warn: ep %d %s was already available",
				     tu_edpt_number(ep->ep_addr),
				     ep_dir_string[tu_edpt_dir(ep->ep_addr)]);
			}
			*ep->buffer_control = value & ~USB_BUF_CTRL_AVAIL;
			// 12 cycle delay.. (should be good for 48*12Mhz = 576Mhz)
			// Don't need delay in host mode as host is in charge
#if !CFG_TUH_ENABLED
			__asm volatile ("b 1f\n"
					"1: b 1f\n"
					"1: b 1f\n"
					"1: b 1f\n"
					"1: b 1f\n"
					"1: b 1f\n" "1:\n":::"memory");
#endif
		}
	}

	*ep->buffer_control = value;
}

// sync endpoint buffer and return transferred bytes
static uint16_t FASE_IRQ sync_ep_buffer(struct hw_endpoint *ep, uint8_t buf_id)
{
	uint32_t buf_ctrl = _hw_endpoint_buffer_control_get_value32(ep);
	if (buf_id)
		buf_ctrl = buf_ctrl >> 16;

	uint16_t xferred_bytes = buf_ctrl & USB_BUF_CTRL_LEN_MASK;

	if (!ep->rx) {
		// We are continuing a transfer here. If we are TX, we have successfully
		// sent some data can increase the length we have sent
		assert(!(buf_ctrl & USB_BUF_CTRL_FULL));

		ep->xferred_len = (uint16_t) (ep->xferred_len + xferred_bytes);
	} else {
		// If we have received some data, so can increase the length
		// we have received AFTER we have copied it to the user buffer at the appropriate offset
		assert(buf_ctrl & USB_BUF_CTRL_FULL);

		memcpy(ep->user_buf, ep->hw_data_buf + buf_id * 64,
		       xferred_bytes);
		ep->xferred_len = (uint16_t) (ep->xferred_len + xferred_bytes);
		ep->user_buf += xferred_bytes;
	}

	// Short packet
	if (xferred_bytes < ep->wMaxPacketSize) {
		pico_trace("  Short packet on buffer %d with %u bytes\n",
			   buf_id, xferred_bytes);
		// Reduce total length as this is last packet
		ep->remaining_len = 0;
	}

	return xferred_bytes;
}

// prepare buffer, return buffer control
static uint32_t FASE_IRQ prepare_ep_buffer(struct hw_endpoint *ep, uint8_t buf_id)
{
	uint16_t const buflen = tu_min16(ep->remaining_len, ep->wMaxPacketSize);
	ep->remaining_len = (uint16_t) (ep->remaining_len - buflen);

	uint32_t buf_ctrl = buflen | USB_BUF_CTRL_AVAIL;

	// PID
	buf_ctrl |=
	    ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
	ep->next_pid ^= 1u;

	if (!ep->rx) {
		// Copy data from user buffer to hw buffer
		memcpy(ep->hw_data_buf + buf_id * 64, ep->user_buf, buflen);
		ep->user_buf += buflen;

		// Mark as full
		buf_ctrl |= USB_BUF_CTRL_FULL;
	}
	// Is this the last buffer? Only really matters for host mode. Will trigger
	// the trans complete irq but also stop it polling. We only really care about
	// trans complete for setup packets being sent
	if (ep->remaining_len == 0) {
		buf_ctrl |= USB_BUF_CTRL_LAST;
	}

	if (buf_id)
		buf_ctrl = buf_ctrl << 16;

	return buf_ctrl;
}

// Prepare buffer control register value
void FASE_IRQ hw_endpoint_start_next_buffer(struct hw_endpoint *ep)
{
	uint32_t ep_ctrl = *ep->endpoint_control;

	// always compute and start with buffer 0
	uint32_t buf_ctrl = prepare_ep_buffer(ep, 0) | USB_BUF_CTRL_SEL;

	// For now: skip double buffered for OUT endpoint in Device mode, since
	// host could send < 64 bytes and cause short packet on buffer0
	// NOTE: this could happen to Host mode IN endpoint
	// Also, Host mode "interrupt" endpoint hardware is only single buffered,
	// NOTE2: Currently Host bulk is implemented using "interrupt" endpoint
	bool const is_host = is_host_mode();
	bool const force_single = (!is_host && !tu_edpt_dir(ep->ep_addr)) ||
	    (is_host && tu_edpt_number(ep->ep_addr) != 0);

	if (ep->remaining_len && !force_single) {
		// Use buffer 1 (double buffered) if there is still data
		// TODO: Isochronous for buffer1 bit-field is different than CBI (control bulk, interrupt)

		buf_ctrl |= prepare_ep_buffer(ep, 1);

		// Set endpoint control double buffered bit if needed
		ep_ctrl &= ~EP_CTRL_INTERRUPT_PER_BUFFER;
		ep_ctrl |=
		    EP_CTRL_DOUBLE_BUFFERED_BITS |
		    EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER;
	} else {
		// Single buffered since 1 is enough
		ep_ctrl &=
		    ~(EP_CTRL_DOUBLE_BUFFERED_BITS |
		      EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER);
		ep_ctrl |= EP_CTRL_INTERRUPT_PER_BUFFER;
	}

	*ep->endpoint_control = ep_ctrl;

	TU_LOG(3, "  Prepare BufCtrl: [0] = 0x%04x  [1] = 0x%04x\r\n",
	       tu_u32_low16(buf_ctrl), tu_u32_high16(buf_ctrl));

	// Finally, write to buffer_control which will trigger the transfer
	// the next time the controller polls this dpram address
	_hw_endpoint_buffer_control_set_value32(ep, buf_ctrl);
}

static void FASE_IRQ _hw_endpoint_xfer_sync(struct hw_endpoint *ep)
{
	// Update hw endpoint struct with info from hardware
	// after a buff status interrupt

	uint32_t __unused buf_ctrl =
	    _hw_endpoint_buffer_control_get_value32(ep);
	TU_LOG(3, "  Sync BufCtrl: [0] = 0x%04x  [1] = 0x%04x\r\n",
	       tu_u32_low16(buf_ctrl), tu_u32_high16(buf_ctrl));

	// always sync buffer 0
	uint16_t buf0_bytes = sync_ep_buffer(ep, 0);

	// sync buffer 1 if double buffered
	if ((*ep->endpoint_control) & EP_CTRL_DOUBLE_BUFFERED_BITS) {
		if (buf0_bytes == ep->wMaxPacketSize) {
			// sync buffer 1 if not short packet
			sync_ep_buffer(ep, 1);
		} else {
			// short packet on buffer 0
			// TODO couldn't figure out how to handle this case which happen with net_lwip_webserver example
			// At this time (currently trigger per 2 buffer), the buffer1 is probably filled with data from
			// the next transfer (not current one). For now we disable double buffered for device OUT
			// NOTE this could happen to Host IN
#if 0
			uint8_t const ep_num = tu_edpt_number(ep->ep_addr);
			uint8_t const dir = (uint8_t) tu_edpt_dir(ep->ep_addr);
			uint8_t const ep_id = 2 * ep_num + (dir ? 0 : 1);

			// abort queued transfer on buffer 1
			usb_hw->abort |= TU_BIT(ep_id);

			while (!(usb_hw->abort_done & TU_BIT(ep_id))) {
			}

			uint32_t ep_ctrl = *ep->endpoint_control;
			ep_ctrl &=
			    ~(EP_CTRL_DOUBLE_BUFFERED_BITS |
			      EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER);
			ep_ctrl |= EP_CTRL_INTERRUPT_PER_BUFFER;

			_hw_endpoint_buffer_control_set_value32(ep, 0);

			usb_hw->abort &= ~TU_BIT(ep_id);

			TU_LOG(3, "----SHORT PACKET buffer0 on EP %02X:\r\n",
			       ep->ep_addr);
			TU_LOG(3, "  BufCtrl: [0] = 0x%04x  [1] = 0x%04x\r\n",
			       tu_u32_low16(buf_ctrl), tu_u32_high16(buf_ctrl));
#endif
		}
	}
}

// Returns true if transfer is complete
bool FASE_IRQ hw_endpoint_xfer_continue(struct hw_endpoint *ep)
{
	hw_endpoint_lock_update(ep, 1);

	// Part way through a transfer
	if (!ep->active) {
		panic("Can't continue xfer on inactive ep %d %s",
		      tu_edpt_number(ep->ep_addr),
		      ep_dir_string[tu_edpt_dir(ep->ep_addr)]);
	}
	// Update EP struct from hardware state
	_hw_endpoint_xfer_sync(ep);

	// Now we have synced our state with the hardware. Is there more data to transfer?
	// If we are done then notify tinyusb
	if (ep->remaining_len == 0) {
		pico_trace("Completed transfer of %d bytes on ep %d %s\n",
			   ep->xferred_len, tu_edpt_number(ep->ep_addr),
			   ep_dir_string[tu_edpt_dir(ep->ep_addr)]);
		// Notify caller we are done so it can notify the tinyusb stack
		hw_endpoint_lock_update(ep, -1);
		return true;
	} else {
		if (e15_is_critical_frame_period(ep)) {
			ep->pending = 1;
		} else {
			hw_endpoint_start_next_buffer(ep);
		}
	}

	hw_endpoint_lock_update(ep, -1);
	// More work to do
	return false;
}

void FASE_IRQ hw_endpoint_xfer_start(struct hw_endpoint *ep, uint8_t * buffer,
			    uint16_t total_len)
{
	hw_endpoint_lock_update(ep, 1);

	if (ep->active) {
		// TODO: Is this acceptable for interrupt packets?
		TU_LOG(1,
		       "WARN: starting new transfer on already active ep %d %s\n",
		       tu_edpt_number(ep->ep_addr),
		       ep_dir_string[tu_edpt_dir(ep->ep_addr)]);
		hw_endpoint_lock_update(ep, -1);
		return;

		hw_endpoint_reset_transfer(ep);
	}
	// Fill in info now that we're kicking off the hw
	ep->remaining_len = total_len;
	ep->xferred_len = 0;
	ep->active = true;
	ep->user_buf = buffer;

	if (e15_is_critical_frame_period(ep)) {
		pico_trace_urb("%s: [%x] pending\n", __func__, ep->ep_addr);
		ep->pending = 1;
	} else {
		hw_endpoint_start_next_buffer(ep);
	}

	if (e15_is_bulkin_ep(ep)) {
		pico_trace_urb("%s: enable sof", __func__);
		usb_hw_set->inte = USB_INTS_DEV_SOF_BITS;
		pico_trace_urb("-> ok\n");
	}

	hw_endpoint_lock_update(ep, -1);
}

static void hw_endpoint_xfer(uint8_t ep_addr, uint8_t * buffer,
			     uint16_t total_bytes)
{
	struct hw_endpoint *ep = hw_endpoint_get_by_addr(ep_addr);
	pico_trace_urb("%s: [%x]->%x\n", __func__, ep_addr, total_bytes);
	hw_endpoint_xfer_start(ep, buffer, total_bytes);
}

static void FASE_IRQ hw_handle_buff_status(void)
{
	uint32_t remaining_buffers = usb_hw->buf_status;
	pico_trace_irq("buf_status = 0x%x\n", remaining_buffers);
	uint bit = 1u;
	for (uint8_t i = 0; remaining_buffers && i < USB_MAX_ENDPOINTS * 2; i++) {
		if (remaining_buffers & bit) {
			// clear this in advance
			usb_hw_clear->buf_status = bit;

			// IN transfer for even i, OUT transfer for odd i
			struct hw_endpoint *ep =
			    hw_endpoint_get_by_num(i >> 1u, !(i & 1u));

			// Continue xfer
			bool done = hw_endpoint_xfer_continue(ep);
			if (done) {
				// Notify
				// dcd_event_xfer_complete(0, ep->ep_addr, ep->xferred_len, XFER_RESULT_SUCCESS, true);
				if (tu_edpt_dir(ep->ep_addr) == TUSB_DIR_IN) {
					if (tu_edpt_number(ep->ep_addr) == 0) {
						hw_endpoint_reset_transfer(ep);
						rt_usbd_ep0_in_handler
						    (&pico_udc);
					} else {
						rt_usbd_ep_in_handler(&pico_udc,
								      ep->
								      ep_addr,
								      ep->
								      xferred_len);
						hw_endpoint_reset_transfer(ep);
					}
				} else {
					if (tu_edpt_number(ep->ep_addr) == 0) {
						rt_usbd_ep0_out_handler
						    (&pico_udc,
						     ep->xferred_len);
						hw_endpoint_reset_transfer(ep);
					} else {
						rt_usbd_ep_out_handler
						    (&pico_udc, ep->ep_addr,
						     ep->xferred_len);
						hw_endpoint_reset_transfer(ep);
					}
				}
			}
			remaining_buffers &= ~bit;
		}
		bit <<= 1u;
	}
}

void FASE_IRQ hw_endpoint_reset_transfer(struct hw_endpoint *ep)
{
	ep->active = false;
	ep->remaining_len = 0;
	ep->xferred_len = 0;
	ep->user_buf = 0;
}

void rp2040_usb_init(void)
{
	// Reset usb controller
	reset_block(RESETS_RESET_USBCTRL_BITS);
	unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

	// Clear any previous state just in case
	memset(usb_hw, 0, sizeof(*usb_hw));
	memset(usb_dpram, 0, sizeof(*usb_dpram));

	// Mux the controller to the onboard usb phy
	usb_hw->muxing =
	    USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;
}

TU_ATTR_ALWAYS_INLINE static inline void reset_ep0_pid(void)
{
	// If we have finished this transfer on EP0 set pid back to 1 for next
	// setup transfer. Also clear a stall in case
	uint8_t addrs[] = { 0x0, 0x80 };
	for (uint i = 0; i < TU_ARRAY_SIZE(addrs); i++) {
		struct hw_endpoint *ep = hw_endpoint_get_by_addr(addrs[i]);
		ep->next_pid = 1u;
	}
}

static void FASE_IRQ dcd_rp2040_irq(void)
{
	uint32_t const status = usb_hw->ints;
	uint32_t handled = 0;

	if (status & USB_INTF_DEV_SOF_BITS) {
		bool keep_sof_alive = false;

		handled |= USB_INTF_DEV_SOF_BITS;

#if TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX
		// Errata 15 workaround for Device Bulk-In endpoint
		e15_last_sof = time_us_32();

		for (uint8_t i = 0; i < USB_MAX_ENDPOINTS; i++) {
			struct hw_endpoint *ep =
			    hw_endpoint_get_by_num(i, TUSB_DIR_IN);

			// Active Bulk IN endpoint requires SOF
			if ((ep->transfer_type == TUSB_XFER_BULK) && ep->active) {
				keep_sof_alive = true;

				hw_endpoint_lock_update(ep, 1);

				// Deferred enable?
				if (ep->pending) {
					ep->pending = 0;
					pico_trace_urb("%s: [%x] continue\n",
						       __func__, ep->ep_addr);
					hw_endpoint_start_next_buffer(ep);
				}

				hw_endpoint_lock_update(ep, -1);
			}
		}
		usb_hw_clear->inte = USB_INTS_DEV_SOF_BITS;
#endif

		// disable SOF interrupt if it is used for RESUME in remote wakeup
		if (!keep_sof_alive && !_sof_enable)
			usb_hw_clear->inte = USB_INTS_DEV_SOF_BITS;

		// dcd_event_sof(0, usb_hw->sof_rd & USB_SOF_RD_BITS, true);
		rt_usbd_sof_handler(&pico_udc);
	}
	// xfer events are handled before setup req. So if a transfer completes immediately
	// before closing the EP, the events will be delivered in same order.
	if (status & USB_INTS_BUFF_STATUS_BITS) {
		handled |= USB_INTS_BUFF_STATUS_BITS;
		hw_handle_buff_status();
	}

	if (status & USB_INTS_SETUP_REQ_BITS) {
		handled |= USB_INTS_SETUP_REQ_BITS;
		uint8_t const *setup =
		    (uint8_t const *)&usb_dpram->setup_packet;

		// reset pid to both 1 (data and ack)
		reset_ep0_pid();

		// Pass setup packet to tiny usb
		// dcd_event_setup_received(0, setup, true);
		rt_usbd_ep0_setup_handler(&pico_udc, (struct urequest *)setup);
		usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
	}
#if FORCE_VBUS_DETECT == 0
	// Since we force VBUS detect On, device will always think it is connected and
	// couldn't distinguish between disconnect and suspend
	if (status & USB_INTS_DEV_CONN_DIS_BITS) {
		handled |= USB_INTS_DEV_CONN_DIS_BITS;

		if (usb_hw->sie_status & USB_SIE_STATUS_CONNECTED_BITS) {
			// Connected: nothing to do
		} else {
			// Disconnected
			dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, true);
		}

		usb_hw_clear->sie_status = USB_SIE_STATUS_CONNECTED_BITS;
	}
#endif
	// SE0 for 2.5 us or more (will last at least 10ms)
	if (status & USB_INTS_BUS_RESET_BITS) {
		pico_trace_irq("BUS RESET\n");

		handled |= USB_INTS_BUS_RESET_BITS;

		usb_hw->dev_addr_ctrl = 0;
		reset_non_control_endpoints();
		// dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
		rt_usbd_reset_handler(&pico_udc);

		usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;

#if TUD_OPT_RP2040_USB_DEVICE_ENUMERATION_FIX
		// Only run enumeration workaround if pull up is enabled
		if (usb_hw->sie_ctrl & USB_SIE_CTRL_PULLUP_EN_BITS)
			rp2040_usb_device_enumeration_fix();
#endif
	}

	/* Note from pico datasheet 4.1.2.6.4 (v1.2)
	 * If you enable the suspend interrupt, it is likely you will see a suspend interrupt when
	 * the device is first connected but the bus is idle. The bus can be idle for a few ms before
	 * the host begins sending start of frame packets. You will also see a suspend interrupt
	 * when the device is disconnected if you do not have a VBUS detect circuit connected. This is
	 * because without VBUS detection, it is impossible to tell the difference between
	 * being disconnected and suspended.
	 */
	if (status & USB_INTS_DEV_SUSPEND_BITS) {
		handled |= USB_INTS_DEV_SUSPEND_BITS;
		// dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
		// TODO Check if need to handle suspend/resume event
		usb_hw_clear->sie_status = USB_SIE_STATUS_SUSPENDED_BITS;
	}

	if (status & USB_INTS_DEV_RESUME_FROM_HOST_BITS) {
		handled |= USB_INTS_DEV_RESUME_FROM_HOST_BITS;
		// dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
		// TODO Check if need to handle suspend/resume event
		usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;
	}

	if (status ^ handled) {
		panic("Unhandled IRQ 0x%x\n", (uint) (status ^ handled));
	}
}

void dcd_init(uint8_t rhport)
{
	// Reset hardware to default state
	rp2040_usb_init();

#if FORCE_VBUS_DETECT
	// Force VBUS detect so the device thinks it is plugged into a host
	usb_hw->pwr =
	    USB_USB_PWR_VBUS_DETECT_BITS |
	    USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
#endif
	irq_add_shared_handler(USBCTRL_IRQ, dcd_rp2040_irq, 0xff);

	// Init control endpoints
	tu_memclr(hw_endpoints[0], 2 * sizeof(hw_endpoint_t));
	hw_endpoint_init(0x0, 64, TUSB_XFER_CONTROL);
	hw_endpoint_init(0x80, 64, TUSB_XFER_CONTROL);

	// Init non-control endpoints
	reset_non_control_endpoints();

	// Initializes the USB peripheral for device mode and enables it.
	// Don't need to enable the pull up here. Force VBUS
	usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

	// Enable individual controller IRQS here. Processor interrupt enable will be used
	// for the global interrupt enable...
	// Note: Force VBUS detect cause disconnection not detectable
	usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;
	usb_hw->inte =
	    USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS |
	    USB_INTS_SETUP_REQ_BITS | USB_INTS_DEV_SUSPEND_BITS |
	    USB_INTS_DEV_RESUME_FROM_HOST_BITS | (FORCE_VBUS_DETECT ? 0 :
						  USB_INTS_DEV_CONN_DIS_BITS);

	dcd_connect(rhport);
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
	(void)rhport;

	if (tu_edpt_number(ep_addr) == 0) {
		// A stall on EP0 has to be armed so it can be cleared on the next setup packet
		usb_hw_set->ep_stall_arm =
		    (tu_edpt_dir(ep_addr) ==
		     TUSB_DIR_IN) ? USB_EP_STALL_ARM_EP0_IN_BITS :
		    USB_EP_STALL_ARM_EP0_OUT_BITS;
	}

	struct hw_endpoint *ep = hw_endpoint_get_by_addr(ep_addr);

	// stall and clear current pending buffer
	// may need to use EP_ABORT
	_hw_endpoint_buffer_control_set_value32(ep, USB_BUF_CTRL_STALL);
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
	(void)rhport;

	if (tu_edpt_number(ep_addr)) {
		struct hw_endpoint *ep = hw_endpoint_get_by_addr(ep_addr);

		// clear stall also reset toggle to DATA0, ready for next transfer
		ep->next_pid = 0;
		_hw_endpoint_buffer_control_clear_mask32(ep,
							 USB_BUF_CTRL_STALL);
	}
}

static rt_err_t _set_address(rt_uint8_t address)
{
	address_setup_seq = true;
	hw_endpoint_xfer(0x80, NULL, 0);
	rt_thread_mdelay(2);
	usb_hw->dev_addr_ctrl = address;
	return RT_EOK;
}

static rt_err_t _set_config(rt_uint8_t address)
{
	return RT_EOK;
}

static rt_err_t _ep_set_stall(rt_uint8_t address)
{
	dcd_edpt_stall(0, address);
	return RT_EOK;
}

static rt_err_t _ep_clear_stall(rt_uint8_t address)
{
	dcd_edpt_clear_stall(0, address);
	return RT_EOK;
}

static rt_err_t _ep_enable(struct uendpoint *ep)
{
	pico_trace_init("%s: %x\n", __func__, ep->ep_desc->bEndpointAddress);
	hw_endpoint_init(ep->ep_desc->bEndpointAddress,
			 ep->ep_desc->wMaxPacketSize,
			 ep->ep_desc->bmAttributes);
	return RT_EOK;
}

static rt_err_t _ep_disable(struct uendpoint *ep)
{
	pico_trace_init("%s: %x\n", __func__, ep->ep_desc->bEndpointAddress);
	hw_endpoint_close(ep->ep_desc->bEndpointAddress);
	return RT_EOK;
}

static rt_size_t _ep_read_prepare(rt_uint8_t address, void *buffer,
				  rt_size_t size)
{
	hw_endpoint_xfer(address, buffer, size);
	return RT_EOK;
}

static rt_size_t _ep_read(rt_uint8_t address, void *buffer)
{
	rt_size_t size = 0;
	RT_ASSERT(buffer != RT_NULL);
	return RT_EOK;
}

static rt_size_t _ep_write(rt_uint8_t address, void *buffer, rt_size_t size)
{
	hw_endpoint_xfer(address, buffer, size);
	return RT_EOK;
}

static rt_err_t _ep0_send_status(void)
{
	// ZLP
	if (address_setup_seq == true) {
		address_setup_seq = false;
		return RT_EOK;
	}
	hw_endpoint_xfer(0x80, NULL, 0x0);
	return RT_EOK;
}

static rt_err_t _suspend(void)
{
	return RT_EOK;
}

static rt_err_t _wakeup(void)
{
	return RT_EOK;
}

const static struct udcd_ops pico_udc_ops = {
	_set_address,
	_set_config,
	_ep_set_stall,
	_ep_clear_stall,
	_ep_enable,
	_ep_disable,
	_ep_read_prepare,
	_ep_read,
	_ep_write,
	_ep0_send_status,
	_suspend,
	_wakeup,
};

static rt_err_t _init(rt_device_t device)
{
	dcd_init(0);
	dcd_int_enable(0);
	return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops _ops = {
	_init,
	RT_NULL,
	RT_NULL,
	RT_NULL,
	RT_NULL,
	RT_NULL,
};
#endif

int pico_usbd_register(void)
{
	rt_memset((void *)&pico_udc, 0, sizeof(struct udcd));
	pico_udc.parent.type = RT_Device_Class_USBDevice;
#ifdef RT_USING_DEVICE_OPS
	pico_udc.parent.ops = &_ops;
#else
	pico_udc.parent.init = _init;
#endif
	// pico_udc.parent.user_data = &pico_pcd;
	pico_udc.ops = &pico_udc_ops;
	/* Register endpoint infomation */
	pico_udc.ep_pool = pico_ep_pool;
	pico_udc.ep0.id = &pico_ep_pool[0];

	rt_device_register((rt_device_t) & pico_udc, "usbd", 0);
	rt_usb_device_init();
	return RT_EOK;
}

INIT_DEVICE_EXPORT(pico_usbd_register);

#endif
