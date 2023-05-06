#ifndef RP2040_COMMON_H_
#define RP2040_COMMON_H_

#include "hardware/structs/usb.h"
#include "hardware/irq.h"
#include "hardware/resets.h"
#include "hardware/timer.h"

// Current implementation force vbus detection as always present, causing device think it is always plugged into host.
// Therefore it cannot detect disconnect event, mistaken it as suspend.
// Note: won't work if change to 0 (for now)
#define FORCE_VBUS_DETECT   1
#define TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX 	1

#define TU_ATTR_ALWAYS_INLINE         __attribute__ ((always_inline))

#define usb_hw_set hw_set_alias(usb_hw)
#define usb_hw_clear hw_clear_alias(usb_hw)

#define pico_info		// rt_kprintf
#define pico_trace		// rt_kprintf
#define pico_trace_init		// rt_kprintf
#define pico_trace_urb		// rt_kprintf
#define pico_trace_irq		// rt_kprintf

#ifndef TU_LOG
#define TU_LOG(n, fmt, ...)	// rt_kprintf(fmt, ##__VA_ARGS__)
#define TU_LOG_MEM(n, ...)
#define TU_LOG_PTR(n, ...)
#define TU_LOG_INT(n, ...)
#define TU_LOG_HEX(n, ...)
#define TU_LOG_LOCATION()
#define TU_LOG_FAILED()
#endif

/*------------------------------------------------------------------*/
/* Macro Generator
 *------------------------------------------------------------------*/

// Helper to implement optional parameter for TU_VERIFY Macro family
#define _GET_3RD_ARG(arg1, arg2, arg3, ...)        arg3
#define _GET_4TH_ARG(arg1, arg2, arg3, arg4, ...)  arg4

/*------------- Generator for TU_VERIFY and TU_VERIFY_HDLR -------------*/
#define TU_VERIFY_DEFINE(_cond, _handler, _ret)  do            \
{                                                              \
  if ( !(_cond) ) { _handler; return _ret;  }                  \
} while(0)

/*------------------------------------------------------------------*/
/* TU_VERIFY
 * - TU_VERIFY_1ARGS : return false if failed
 * - TU_VERIFY_2ARGS : return provided value if failed
 *------------------------------------------------------------------*/
#define TU_VERIFY_1ARGS(_cond)                         TU_VERIFY_DEFINE(_cond, , false)
#define TU_VERIFY_2ARGS(_cond, _ret)                   TU_VERIFY_DEFINE(_cond, , _ret)

#define TU_VERIFY(...)                   _GET_3RD_ARG(__VA_ARGS__, TU_VERIFY_2ARGS, TU_VERIFY_1ARGS, UNUSED)(__VA_ARGS__)

//------------- Mem -------------//
#define tu_memclr(buffer, size)  memset((buffer), 0, (size))
#define tu_varclr(_var)          tu_memclr(_var, sizeof(*(_var)))

//--------------------------------------------------------------------+
// Macros Helper
//--------------------------------------------------------------------+
#define TU_ARRAY_SIZE(_arr)   ( sizeof(_arr) / sizeof(_arr[0]) )
#define TU_MIN(_x, _y)        ( ( (_x) < (_y) ) ? (_x) : (_y) )
#define TU_MAX(_x, _y)        ( ( (_x) > (_y) ) ? (_x) : (_y) )

#define TU_U16(_high, _low)   ((uint16_t) (((_high) << 8) | (_low)))
#define TU_U16_HIGH(_u16)     ((uint8_t) (((_u16) >> 8) & 0x00ff))
#define TU_U16_LOW(_u16)      ((uint8_t) ((_u16)       & 0x00ff))
#define U16_TO_U8S_BE(_u16)   TU_U16_HIGH(_u16), TU_U16_LOW(_u16)
#define U16_TO_U8S_LE(_u16)   TU_U16_LOW(_u16), TU_U16_HIGH(_u16)

#define TU_U32_BYTE3(_u32)    ((uint8_t) ((((uint32_t) _u32) >> 24) & 0x000000ff))	// MSB
#define TU_U32_BYTE2(_u32)    ((uint8_t) ((((uint32_t) _u32) >> 16) & 0x000000ff))
#define TU_U32_BYTE1(_u32)    ((uint8_t) ((((uint32_t) _u32) >>  8) & 0x000000ff))
#define TU_U32_BYTE0(_u32)    ((uint8_t) (((uint32_t)  _u32)        & 0x000000ff))	// LSB

#define U32_TO_U8S_BE(_u32)   TU_U32_BYTE3(_u32), TU_U32_BYTE2(_u32), TU_U32_BYTE1(_u32), TU_U32_BYTE0(_u32)
#define U32_TO_U8S_LE(_u32)   TU_U32_BYTE0(_u32), TU_U32_BYTE1(_u32), TU_U32_BYTE2(_u32), TU_U32_BYTE3(_u32)

#define TU_BIT(n)             (1UL << (n))
#define TU_GENMASK(h, l)      ( (UINT32_MAX << (l)) & (UINT32_MAX >> (31 - (h))) )

/// defined base on USB Specs Endpoint's bmAttributes
typedef enum {
	TUSB_XFER_CONTROL = 0,
	TUSB_XFER_ISOCHRONOUS,
	TUSB_XFER_BULK,
	TUSB_XFER_INTERRUPT
} tusb_xfer_type_t;

typedef enum {
	TUSB_DIR_OUT = 0,
	TUSB_DIR_IN = 1,

	TUSB_DIR_IN_MASK = 0x80
} tusb_dir_t;

// Hardware information per endpoint
typedef struct hw_endpoint {
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
	io_rw_32 *buffer_control;

	// Buffer pointer in usb dpram
	uint8_t *hw_data_buf;

	// User buffer in main memory
	uint8_t *user_buf;

	// Current transfer information
	uint16_t remaining_len;
	uint16_t xferred_len;

	// Data needed from EP descriptor
	uint16_t wMaxPacketSize;

	// Endpoint is in use
	bool active;

	// Interrupt, bulk, etc
	uint8_t transfer_type;
	// Transfer scheduled but not active

	uint8_t pending;

} hw_endpoint_t;

TU_ATTR_ALWAYS_INLINE static inline tusb_dir_t tu_edpt_dir(uint8_t addr)
{
	return (addr & TUSB_DIR_IN_MASK) ? TUSB_DIR_IN : TUSB_DIR_OUT;
}

TU_ATTR_ALWAYS_INLINE static inline uint8_t tu_edpt_number(uint8_t addr)
{
	return (uint8_t) (addr & (~TUSB_DIR_IN_MASK));
}

TU_ATTR_ALWAYS_INLINE static inline uint8_t tu_u32_byte3(uint32_t ui32)
{
	return TU_U32_BYTE3(ui32);
}

TU_ATTR_ALWAYS_INLINE static inline uint8_t tu_u32_byte2(uint32_t ui32)
{
	return TU_U32_BYTE2(ui32);
}

TU_ATTR_ALWAYS_INLINE static inline uint8_t tu_u32_byte1(uint32_t ui32)
{
	return TU_U32_BYTE1(ui32);
}

TU_ATTR_ALWAYS_INLINE static inline uint8_t tu_u32_byte0(uint32_t ui32)
{
	return TU_U32_BYTE0(ui32);
}

TU_ATTR_ALWAYS_INLINE static inline uint16_t tu_u32_high16(uint32_t ui32)
{
	return (uint16_t) (ui32 >> 16);
}

TU_ATTR_ALWAYS_INLINE static inline uint16_t tu_u32_low16(uint32_t ui32)
{
	return (uint16_t) (ui32 & 0x0000ffffu);
}

TU_ATTR_ALWAYS_INLINE static inline uint8_t tu_u16_high(uint16_t ui16)
{
	return TU_U16_HIGH(ui16);
}

TU_ATTR_ALWAYS_INLINE static inline uint8_t tu_u16_low(uint16_t ui16)
{
	return TU_U16_LOW(ui16);
}

//------------- Bits -------------//
TU_ATTR_ALWAYS_INLINE static inline uint32_t tu_bit_set(uint32_t value,
							uint8_t pos)
{
	return value | TU_BIT(pos);
}

TU_ATTR_ALWAYS_INLINE static inline uint32_t tu_bit_clear(uint32_t value,
							  uint8_t pos)
{
	return value & (~TU_BIT(pos));
}

TU_ATTR_ALWAYS_INLINE static inline bool tu_bit_test(uint32_t value,
						     uint8_t pos)
{
	return (value & TU_BIT(pos)) ? true : false;
}

//------------- Min -------------//
TU_ATTR_ALWAYS_INLINE static inline uint8_t tu_min8(uint8_t x, uint8_t y)
{
	return (x < y) ? x : y;
}

TU_ATTR_ALWAYS_INLINE static inline uint16_t tu_min16(uint16_t x, uint16_t y)
{
	return (x < y) ? x : y;
}

TU_ATTR_ALWAYS_INLINE static inline uint32_t tu_min32(uint32_t x, uint32_t y)
{
	return (x < y) ? x : y;
}

//------------- Max -------------//
TU_ATTR_ALWAYS_INLINE static inline uint8_t tu_max8(uint8_t x, uint8_t y)
{
	return (x > y) ? x : y;
}

TU_ATTR_ALWAYS_INLINE static inline uint16_t tu_max16(uint16_t x, uint16_t y)
{
	return (x > y) ? x : y;
}

TU_ATTR_ALWAYS_INLINE static inline uint32_t tu_max32(uint32_t x, uint32_t y)
{
	return (x > y) ? x : y;
}

//------------- Mathematics -------------//
TU_ATTR_ALWAYS_INLINE static inline uint32_t tu_div_ceil(uint32_t v, uint32_t d)
{
	return (v + d - 1) / d;
}

void _hw_endpoint_buffer_control_update32(struct hw_endpoint *ep,
					  uint32_t and_mask, uint32_t or_mask);

TU_ATTR_ALWAYS_INLINE static inline uint32_t
_hw_endpoint_buffer_control_get_value32(struct hw_endpoint *ep)
{
	return *ep->buffer_control;
}

TU_ATTR_ALWAYS_INLINE static inline void
_hw_endpoint_buffer_control_set_value32(struct hw_endpoint *ep, uint32_t value)
{
	return _hw_endpoint_buffer_control_update32(ep, 0, value);
}

TU_ATTR_ALWAYS_INLINE static inline void
_hw_endpoint_buffer_control_set_mask32(struct hw_endpoint *ep, uint32_t value)
{
	return _hw_endpoint_buffer_control_update32(ep, ~value, value);
}

TU_ATTR_ALWAYS_INLINE static inline void
_hw_endpoint_buffer_control_clear_mask32(struct hw_endpoint *ep, uint32_t value)
{
	return _hw_endpoint_buffer_control_update32(ep, ~value, 0);
}

static inline uintptr_t hw_data_offset(uint8_t * buf)
{
	// Remove usb base from buffer pointer
	return (uintptr_t) buf ^ (uintptr_t) usb_dpram;
}

TU_ATTR_ALWAYS_INLINE static inline void hw_endpoint_lock_update(__unused struct
								 hw_endpoint
								 *ep,
								 __unused int
								 delta)
{
	// todo add critsec as necessary to prevent issues between worker and IRQ...
	//  note that this is perhaps as simple as disabling IRQs because it would make
	//  sense to have worker and IRQ on same core, however I think using critsec is about equivalent.
}

// Function Prototype
void hw_endpoint_reset_transfer(struct hw_endpoint *ep);

#endif
