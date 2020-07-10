/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2020 Lubomir Rintel <lkundrak@v3.sk>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <string.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/desig.h>

#include "gud.h"
#include "st7789.h"
#include "config.h"

#ifdef GD32VF103
#define USB_DRIVER otgfs_usb_driver
#else
#define USB_DRIVER st_usbfs_v1_usb_driver
#endif

#define EP0_MAX_LEN	64
#define EP1_MAX_LEN	64

/* Do not enable this. UART I/O seems to interfere with USB timing. */
#ifndef DEBUG
#define printf(...)
#endif

enum {
	STR_MANUFACTURER,
	STR_PRODUCT,
	STR_SERIAL,
};

static char str_serial[25];

static const char *usb_strings[] = {
	[STR_MANUFACTURER] = "Lubomir Rintel <lkundrak@v3.sk>",
	[STR_PRODUCT] = "LCD Display Interface",
	[STR_SERIAL] = str_serial
};

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0, /* USB_CLASS_PER_INTERFACE */
	.bDeviceSubClass = 0x00,
	.bDeviceProtocol = 0x00,
	.bMaxPacketSize0 = EP0_MAX_LEN,
	.idVendor = 0x1d50,
	.idProduct = 0x614d,
	.bcdDevice = 0x0100,
	.iManufacturer = STR_MANUFACTURER + 1,
	.iProduct = STR_PRODUCT + 1,
	.iSerialNumber = STR_SERIAL + 1,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_ENDPOINT_ADDR_OUT(1),
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = EP1_MAX_LEN,
}};

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_VENDOR,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = USB_CONFIG_ATTR_DEFAULT,
	.bMaxPower = 100 / 2, /* 100 mA, in units of 2 mA */

	.interface = ifaces,
};

#define GUD_DISPLAY_MAGIC                       0x1d50614d

struct gud_display_descriptor_req {
        uint32_t magic;
        uint8_t version;
        uint32_t flags;
        uint8_t compression;
        uint32_t max_buffer_size;
        uint32_t min_width;
        uint32_t max_width;
        uint32_t min_height;
        uint32_t max_height;
        uint8_t num_formats;
        uint8_t num_properties;
        uint8_t num_connectors;
} __attribute__((packed));

struct gud_display_descriptor_req display_desc = {
	.magic = GUD_DISPLAY_MAGIC,
	.version = 1,
	.flags = 0,
	.compression = 0,
	.max_buffer_size = 0,
	.min_width = DISPLAY_COLS,
	.max_width = DISPLAY_COLS,
	.min_height = DISPLAY_ROWS,
	.max_height = DISPLAY_ROWS,
	.num_formats = 1,
	.num_properties = 0,
	.num_connectors = 1,
};

#define GUD_PIXEL_FORMAT_RGB565               0x40

static const uint32_t pixel_formats[] = {
	GUD_PIXEL_FORMAT_RGB565,
};

#define GUD_CONNECTOR_TYPE_PANEL                0

struct gud_drm_req_get_connector {
	uint8_t connector_type;
	uint32_t flags;
	uint8_t num_properties;
} __attribute__((packed));

struct gud_drm_req_get_connector conn = {
	.connector_type = GUD_CONNECTOR_TYPE_PANEL,
	.flags = 0,
	.num_properties = 0,
};

#define GUD_CONNECTOR_STATUS_CONNECTED          0x01

struct gud_drm_req_get_connector_status {
	uint8_t status;
	uint16_t num_modes;
	uint16_t edid_len;
} __attribute__((packed));

struct gud_drm_req_get_connector_status conn_stat = {
	.status = GUD_CONNECTOR_STATUS_CONNECTED,
	.num_modes = 1,
	.edid_len = 0,
};

#define GUD_DISPLAY_MODE_FLAG_PREFERRED		0x400

struct gud_drm_display_mode {
	uint32_t clock;
	uint16_t hdisplay;
	uint16_t hsync_start;
	uint16_t hsync_end;
	uint16_t htotal;
	uint16_t vdisplay;
	uint16_t vsync_start;
	uint16_t vsync_end;
	uint16_t vtotal;
	uint32_t flags;
} __attribute__((packed));

struct gud_drm_display_mode modes[] = {{
	.clock = 1,
	.hdisplay = DISPLAY_COLS,
	.hsync_start = DISPLAY_COLS,
	.hsync_end = DISPLAY_COLS,
	.htotal = DISPLAY_COLS,
	.vdisplay = DISPLAY_ROWS,
	.vsync_start = DISPLAY_ROWS,
	.vsync_end = DISPLAY_ROWS,
	.vtotal = DISPLAY_ROWS,
	.flags = GUD_DISPLAY_MODE_FLAG_PREFERRED,
}};

struct gud_req_set_buffer {
	uint32_t x;
	uint32_t y;
	uint32_t width;
	uint32_t height;

	uint32_t length;
	uint8_t compression;
	uint32_t compressed_length;
} __attribute__((packed));

#define GUD_REQ_GET_STATUS                              0x00
#define GUD_REQ_GET_DESCRIPTOR                          0x01
#define GUD_REQ_GET_FORMATS                             0x40
#define GUD_REQ_GET_CONNECTOR                           0x50
#define GUD_REQ_SET_CONNECTOR_FORCE_DETECT              0x53
#define GUD_REQ_GET_CONNECTOR_STATUS                    0x54
#define GUD_REQ_GET_CONNECTOR_MODES                     0x55
#define GUD_REQ_SET_BUFFER                              0x60
#define GUD_REQ_SET_STATE_CHECK                         0x61
#define GUD_REQ_SET_STATE_COMMIT                        0x62
#define GUD_REQ_SET_CONTROLLER_ENABLE                   0x63
#define GUD_REQ_SET_DISPLAY_ENABLE                      0x64

#define GUD_STATUS_OK                         0x00

#define RETURN_BUF(ret_buf)				\
		*buf = (uint8_t *)&ret_buf;		\
		if (*len > sizeof(ret_buf))		\
			*len = sizeof(ret_buf);		\
		return USBD_REQ_HANDLED

static enum usbd_request_return_codes control_request(
		usbd_device *usbd_dev,
		struct usb_setup_data *req,
		uint8_t **buf,
		uint16_t *len,
		void (**complete)(usbd_device *usbd_dev,
				  struct usb_setup_data *req))
{
	struct gud_req_set_buffer *set_buffer;

	(void)usbd_dev;
	(void)complete;

	printf ("CONTROL bRequest=0x%02x len=%d\n", req->bRequest, *len);

	switch (req->bRequest) {

	case GUD_REQ_GET_STATUS:
		memset(*buf, GUD_STATUS_OK, *len);
		return USBD_REQ_HANDLED;

	case GUD_REQ_GET_DESCRIPTOR:
		RETURN_BUF(display_desc);

	case GUD_REQ_GET_FORMATS:
		RETURN_BUF(pixel_formats);

	case GUD_REQ_GET_CONNECTOR:
		RETURN_BUF(conn);

	case GUD_REQ_SET_CONNECTOR_FORCE_DETECT:
		return USBD_REQ_HANDLED;

	case GUD_REQ_GET_CONNECTOR_STATUS:
		RETURN_BUF(conn_stat);

	case GUD_REQ_GET_CONNECTOR_MODES:
		RETURN_BUF(modes);

	case GUD_REQ_SET_BUFFER:
		if (*len < sizeof(struct gud_req_set_buffer))
			return USBD_REQ_NOTSUPP;

		set_buffer = (struct gud_req_set_buffer *)*buf;

		printf(" SET_BUFFER x=%ld y=%ld "
		       "width=%ld height=%ld "
		       "length=%ld "
		       "compression=%d compressed_length=%ld\n",
		       set_buffer->x, set_buffer->y,
		       set_buffer->width, set_buffer->height,
		       set_buffer->length,
		       set_buffer->compression, set_buffer->compressed_length);

		/* FIXME: This only works for if we're starting at column zero! */
		st7789_set_addrs(set_buffer->x, set_buffer->y,
				 set_buffer->x + set_buffer->width - 1,
				 set_buffer->y + set_buffer->height - 1);
		return USBD_REQ_HANDLED;

	/* Not implemented, but no big deal. */
	case GUD_REQ_SET_STATE_CHECK:
	case GUD_REQ_SET_STATE_COMMIT:
	case GUD_REQ_SET_CONTROLLER_ENABLE:
	case GUD_REQ_SET_DISPLAY_ENABLE:
		return USBD_REQ_HANDLED;

	default:
		printf ("CONTROL bRequest=0x%02x len=%d\n", req->bRequest, *len);
		return USBD_REQ_NOTSUPP;
	}
}

#undef RETURN_BUF

static void data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	uint8_t buf[EP1_MAX_LEN];
	int len;

	len = usbd_ep_read_packet(usbd_dev, ep, buf, sizeof(buf));

	printf("DATA ep=0x%02x len=%d\n", ep, len);
	if (ep == USB_ENDPOINT_ADDR_OUT(1))
		st7789_send_data(buf, len);
}

static void set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	if (wValue != 1)
		return;

	usbd_register_control_callback(
			usbd_dev,
			USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE,
			USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
			control_request);

	usbd_ep_setup(usbd_dev,
		      USB_ENDPOINT_ADDR_OUT(1),
		      USB_ENDPOINT_ATTR_BULK,
		      EP1_MAX_LEN,
		      data_rx_cb);
}

uint8_t usbd_control_buffer[EP0_MAX_LEN];
usbd_device *gud_dev;

void gud_init(void)
{
	desig_get_unique_id_as_string(str_serial, sizeof(str_serial));

	gud_dev = usbd_init(&USB_DRIVER, &dev, &config,
		usb_strings, STR_SERIAL + 1,
		usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(gud_dev, set_config);
}

void gud_poll(void)
{
	usbd_poll(gud_dev);
}
