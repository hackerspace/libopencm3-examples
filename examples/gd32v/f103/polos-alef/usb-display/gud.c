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
#include "ssd1306.h"
#include "config.h"

#ifdef GD32VF103
#define USB_DRIVER otgfs_usb_driver
#else
#define USB_DRIVER st_usbfs_v1_usb_driver
#endif

#define EP0_MAX_LEN	64
#define EP1_MAX_LEN	64

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
	[STR_PRODUCT] = "OLED Display Interface",
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

#define GUD_DRM_USB_DT_DISPLAY		(USB_REQ_TYPE_VENDOR | 0x4)

struct gud_drm_display_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;

	uint8_t bVersion;
	uint8_t bMaxBufferSizeOrder;
	uint32_t bmFlags;

	uint8_t bCompression;

	uint32_t dwMinWidth;
	uint32_t dwMaxWidth;
	uint32_t dwMinHeight;
	uint32_t dwMaxHeight;

	uint8_t bNumFormats;
	uint8_t bNumProperties;
	uint8_t bNumConnectors;
} __attribute__((packed));

struct gud_drm_display_descriptor display_desc = {
	.bLength = sizeof(struct gud_drm_display_descriptor),
	.bDescriptorType = (USB_REQ_TYPE_VENDOR | 0x4),
	.bVersion = 1,
	.bMaxBufferSizeOrder = 10,
	.bmFlags = 0,
	.bCompression = 0,
	.dwMinWidth = SSD1306_COLS,
	.dwMaxWidth = SSD1306_COLS,
	.dwMinHeight = SSD1306_ROWS,
	.dwMaxHeight = SSD1306_ROWS,
	.bNumFormats = 1,
	.bNumProperties = 0,
	.bNumConnectors = 1,
};

#define GUD_DRM_FORMAT_R1		0x20203152	/* le32("R1  ") */

static const uint32_t pixel_formats[] = {
	GUD_DRM_FORMAT_R1,
};

#define DRM_MODE_CONNECTOR_SPI		19

struct gud_drm_req_get_connector {
	uint8_t connector_type;
	uint32_t flags;
	uint8_t num_properties;
} __attribute__((packed));

struct gud_drm_req_get_connector conn = {
	.connector_type = DRM_MODE_CONNECTOR_SPI,
	.flags = 0,
	.num_properties = 0,
};

#define connector_status_connected	1

struct gud_drm_req_get_connector_status {
	uint8_t status;
	uint16_t num_modes;
	uint16_t edid_len;
} __attribute__((packed));

struct gud_drm_req_get_connector_status conn_stat = {
	.status = connector_status_connected,
	.num_modes = 1,
	.edid_len = 0,
};

struct gud_drm_display_mode {
	uint32_t clock;
	uint16_t hdisplay;
	uint16_t hsync_start;
	uint16_t hsync_end;
	uint16_t htotal;
	uint16_t hskew;
	uint16_t vdisplay;
	uint16_t vsync_start;
	uint16_t vsync_end;
	uint16_t vtotal;
	uint16_t vscan;
	uint32_t flags;
	uint8_t type;
} __attribute__((packed));

struct gud_drm_display_mode modes[] = {{
	.clock = 1,
	.hdisplay = SSD1306_COLS,
	.hsync_start = SSD1306_COLS,
	.hsync_end = SSD1306_COLS,
	.htotal = SSD1306_COLS,
	.hskew = 0,
	.vdisplay = SSD1306_ROWS,
	.vsync_start = SSD1306_ROWS,
	.vsync_end = SSD1306_ROWS,
	.vtotal = SSD1306_ROWS,
	.vscan = 0,
	.flags = 0,
	.type = 0,
}};

struct gud_drm_req_set_buffer {
	uint32_t x;
	uint32_t y;
	uint32_t width;
	uint32_t height;

	uint32_t length;
	uint8_t compression;
	uint32_t compressed_length;
} __attribute__((packed));

#define GUD_DRM_USB_REQ_GET_FORMATS			0x40
#define GUD_DRM_USB_REQ_GET_CONNECTOR			0x50
#define GUD_DRM_USB_REQ_SET_CONNECTOR_FORCE_DETECT	0x53
#define GUD_DRM_USB_REQ_GET_CONNECTOR_STATUS		0x54
#define GUD_DRM_USB_REQ_GET_CONNECTOR_MODES		0x55
#define GUD_DRM_USB_REQ_SET_BUFFER			0x60

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
	struct gud_drm_req_set_buffer *set_buffer;

	(void)usbd_dev;
	(void)complete;

	printf ("CONTROL bRequest=0x%02x len=%d\n", req->bRequest, *len);

	switch (req->bRequest) {

	case USB_REQ_GET_STATUS:
		/* We get this after SET commands. Not sure why. */
		memset(*buf, 0, *len);
		return USBD_REQ_HANDLED;

	case USB_REQ_GET_DESCRIPTOR:
		RETURN_BUF(display_desc);

	case GUD_DRM_USB_REQ_GET_FORMATS:
		RETURN_BUF(pixel_formats);

	case GUD_DRM_USB_REQ_GET_CONNECTOR:
		RETURN_BUF(conn);

	case GUD_DRM_USB_REQ_SET_CONNECTOR_FORCE_DETECT:
		return USBD_REQ_HANDLED;

	case GUD_DRM_USB_REQ_GET_CONNECTOR_STATUS:
		RETURN_BUF(conn_stat);

	case GUD_DRM_USB_REQ_GET_CONNECTOR_MODES:
		RETURN_BUF(modes);

	case GUD_DRM_USB_REQ_SET_BUFFER:
		if (*len < sizeof(struct gud_drm_req_set_buffer))
			return USBD_REQ_NOTSUPP;

		set_buffer = (struct gud_drm_req_set_buffer *)*buf;

		printf(" SET_BUFFER x=%ld y=%ld "
		       "width=%ld height=%ld "
		       "length=%ld "
		       "compression=%d compressed_length=%ld\n",
		       set_buffer->x, set_buffer->y,
		       set_buffer->width, set_buffer->height,
		       set_buffer->length,
		       set_buffer->compression, set_buffer->compressed_length);

		/* FIXME: This only works for if we're starting at column zero! */
		ssd1306_set_addrs(set_buffer->y, 0);
		return USBD_REQ_HANDLED;

	default:
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
		ssd1306_send_data(buf, len);
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
