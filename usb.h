#ifndef __USB_H
#define __USB_H
#include <libusb.h>

typedef struct usb_context_s {
	libusb_device_handle *dev_handle;
	libusb_context *ctx;
} usb_context_t;

bool usb_init(programmer_t *pgm, unsigned int vid, unsigned int pid);
void usb_close(programmer_t *pgm);

#define DEV_HANDLE(pgm) (((usb_context_t*)(pgm->ctx))->dev_handle)

#endif
