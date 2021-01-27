#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <stdbool.h>
#include <assert.h>
#include "pgm.h"
#include "usb.h"

bool usb_init(programmer_t *pgm, unsigned int vid, unsigned int pid) {
	libusb_device **devs;
	libusb_context *ctx = NULL;

	int r;
	ssize_t cnt;
	r = libusb_init(&ctx);
	if(r < 0) return(false);

// xiaolaba 2020-01-27
//	usb.c:20:2: warning: ‘libusb_set_debug’ is deprecated: Use libusb_set_option instead [-Wdeprecated-declarations]
//  	libusb_set_debug(ctx, 3);
//	libusb_set_debug(ctx, 3);
	libusb_set_option(ctx, 3);	
	cnt = libusb_get_device_list(ctx, &devs);
	if(cnt < 0) return(false);

	usb_context_t *uctx = malloc(sizeof(usb_context_t));
	uctx->dev_handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
	assert(uctx->dev_handle);
	uctx->ctx = ctx;
	pgm->ctx = uctx;

	libusb_free_device_list(devs, 1); //free the list, unref the devices in it

	if(libusb_kernel_driver_active(uctx->dev_handle, 0) == 1) { //find out if kernel driver is attached
		int r = libusb_detach_kernel_driver(uctx->dev_handle, 0);
		assert(r == 0);
	}

#ifdef __APPLE__
	r = libusb_claim_interface(uctx->dev_handle, 0);
	assert(r == 0);
#endif

	return(true);
}

void usb_close(programmer_t *pgm)
{
	// TODO release resources
}
