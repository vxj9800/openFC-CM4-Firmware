/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef USB_HAL_H_
#define USB_HAL_H_

// For memcpy
#include <string.h>
// Include descriptor struct definitions
#include "usb_common.h"
// USB register definitions from pico-sdk
#include "hardware/regs/usb.h"
// USB hardware struct definitions from pico-sdk
#include "hardware/structs/usb.h"
// For interrupt enable and numbers
#include "hardware/irq.h"
// For resetting the USB controller
#include "hardware/resets.h"

typedef void (*usb_ep_handler)(uint8_t *buf, uint16_t len);
typedef void (*usb_setup_handler)(struct usb_setup_packet *pkt);
typedef void (*usb_bus_reset_handler)(void);

// Struct in which we keep the endpoint configuration
struct usb_endpoint_configuration
{
	const struct usb_endpoint_descriptor *descriptor;
	usb_ep_handler handler;

	// Pointers to endpoint + buffer control registers
	// in the USB controller DPSRAM
	volatile uint32_t *endpoint_control;
	volatile uint32_t *buffer_control;
	volatile uint8_t *data_buffer;

	// Toggle after each packet (unless replying to a SETUP)
	uint8_t next_pid;
};

// Struct in which we keep the device configuration
struct usb_device_configuration
{
	const struct usb_device_descriptor *device_descriptor;
	const struct usb_interface_descriptor *interface_descriptor;
	const struct usb_configuration_descriptor *config_descriptor;
	const unsigned char *lang_descriptor;
	const unsigned char **descriptor_strings;
	// Function to handle bus reset
	usb_bus_reset_handler busResetHandler;
	// Function to handle setup packets
	usb_setup_handler setupPktHandler;
	// USB num endpoints is 16
	struct usb_endpoint_configuration endpoints[USB_NUM_ENDPOINTS];
};
extern struct usb_device_configuration dev_config;

#define USB_DIR_IN 0x80u
#define USB_DIR_OUT 0x00u
#define EP0_IN_ADDR (USB_DIR_IN | 0)
#define EP0_OUT_ADDR (USB_DIR_OUT | 0)
#define EP1_IN_ADDR (USB_DIR_IN | 1)
#define EP1_OUT_ADDR (USB_DIR_OUT | 1)
#define EP2_IN_ADDR (USB_DIR_IN | 2)
#define EP2_OUT_ADDR (USB_DIR_OUT | 2)
#define EP3_IN_ADDR (USB_DIR_IN | 3)
#define EP3_OUT_ADDR (USB_DIR_OUT | 3)
#define EP4_IN_ADDR (USB_DIR_IN | 4)
#define EP4_OUT_ADDR (USB_DIR_OUT | 4)
#define EP5_IN_ADDR (USB_DIR_IN | 5)
#define EP5_OUT_ADDR (USB_DIR_OUT | 5)
#define EP6_IN_ADDR (USB_DIR_IN | 6)
#define EP6_OUT_ADDR (USB_DIR_OUT | 6)
#define EP7_IN_ADDR (USB_DIR_IN | 7)
#define EP7_OUT_ADDR (USB_DIR_OUT | 7)
#define EP8_IN_ADDR (USB_DIR_IN | 8)
#define EP8_OUT_ADDR (USB_DIR_OUT | 8)
#define EP9_IN_ADDR (USB_DIR_IN | 9)
#define EP9_OUT_ADDR (USB_DIR_OUT | 9)
#define EP10_IN_ADDR (USB_DIR_IN | 10)
#define EP10_OUT_ADDR (USB_DIR_OUT | 10)
#define EP11_IN_ADDR (USB_DIR_IN | 11)
#define EP11_OUT_ADDR (USB_DIR_OUT | 11)
#define EP12_IN_ADDR (USB_DIR_IN | 12)
#define EP12_OUT_ADDR (USB_DIR_OUT | 12)
#define EP13_IN_ADDR (USB_DIR_IN | 13)
#define EP13_OUT_ADDR (USB_DIR_OUT | 13)
#define EP14_IN_ADDR (USB_DIR_IN | 14)
#define EP14_OUT_ADDR (USB_DIR_OUT | 14)
#define EP15_IN_ADDR (USB_DIR_IN | 15)
#define EP15_OUT_ADDR (USB_DIR_OUT | 15)

// Macros for register operations
#define usb_hw_set hw_set_alias(usb_hw)
#define usb_hw_clear hw_clear_alias(usb_hw)

// Necessary USB functions
void usb_device_init();
struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr);
void usb_start_transfer(struct usb_endpoint_configuration *ep, uint8_t *buf, uint16_t len);
void send_stall_condition(struct usb_endpoint_configuration *ep);

/**
 * @brief Set the assigned device address in the USB peripheral
 *
 */
inline void usb_set_dev_addr(uint8_t dev_addr)
{
	usb_hw->dev_addr_ctrl = dev_addr;
}

#endif