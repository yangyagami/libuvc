/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2010-2012 Ken Tossell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/**
\mainpage libuvc: a cross-platform library for USB video devices

\b libuvc is a library that supports enumeration, control and streaming
for USB Video Class (UVC) devices, such as consumer webcams.

\section features Features
\li UVC device \ref device "discovery and management" API
\li \ref streaming "Video streaming" (device to host) with asynchronous/callback and synchronous/polling modes
\li Read/write access to standard \ref ctrl "device settings"
\li \ref frame "Conversion" between various formats: RGB, YUV, JPEG, etc.
\li Tested on Mac and Linux, portable to Windows and some BSDs

\section roadmap Roadmap
\li Bulk-mode image capture
\li One-shot image capture
\li Improved support for standard settings
\li Support for "extended" (vendor-defined) settings

\section misc Misc.
\p The source code can be found at https://github.com/libuvc/libuvc/. To build
the library, install <a href="https://libusb.info/">libusb</a> 1.0+ and run:

\code
$ git clone https://github.com/libuvc/libuvc.git
$ cd libuvc
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make && make install
\endcode

\section Example
In this example, libuvc is used to acquire images in a 30 fps, 640x480
YUV stream from a UVC device such as a standard webcam.

\include example.c

*/

/**
 * @defgroup init Library initialization/deinitialization
 * @brief Setup routines used to construct UVC access contexts
 */
#include <assert.h>
#include <pthread.h>

#include "libuvc/libuvc.h"
#include "libuvc/libuvc_internal.h"
#include "usb_event_queue.h"

/** @internal
 * @brief Hotplug callback.
 */
int _uvc_hotplug_callback(
    struct libusb_context *ctx,
    struct libusb_device *dev,
    libusb_hotplug_event event,
    void *user_data) {
  static libusb_device_handle *dev_handle = NULL;
  struct libusb_device_descriptor desc;

  uvc_context_t *uvc_ctx = (uvc_context_t *) user_data;

  (void)libusb_get_device_descriptor(dev, &desc);

  if (LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT == event) {
    UVC_DEBUG("USB LEFT EVENT!!!!, tid: %ld", pthread_self());

    if (!uvc_usb_event_queue_isfull(uvc_ctx->queue)) {
      uvc_usb_event_t e = {
      	    .vid = desc.idVendor,
      	    .pid = desc.idProduct,
      	    .type = LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT
      };
      uvc_usb_event_queue_enqueue(uvc_ctx->queue, e);
    }
  } else {
    UVC_DEBUG("Unhandled event %d\n", event);
  }

  return 0;
}


/** @internal
 * @brief Event handler thread
 * There's one of these per UVC context.
 * @todo We shouldn't run this if we don't own the USB context
 */
void *_uvc_handle_events(void *arg) {
  uvc_context_t *ctx = (uvc_context_t *) arg;

  while (!ctx->kill_handler_thread) {
    if (!uvc_usb_event_queue_empty(ctx->queue)) {
      uvc_device_handle_t *devh = NULL;
      uvc_usb_event_t e = uvc_usb_event_queue_dequeue(ctx->queue);

      DL_FOREACH(ctx->open_devices, devh) {
        assert(devh != NULL && devh->dev != NULL && devh->dev->usb_dev != NULL);

        struct libusb_device_descriptor open_device_desc;
        (void)libusb_get_device_descriptor(devh->dev->usb_dev,
                                           &open_device_desc);
        if (open_device_desc.idVendor == e.vid &&
            open_device_desc.idProduct == e.pid) {
          if (uvc_device_opened(ctx, devh)) {
      	    uvc_close(ctx, devh);
      	    UVC_DEBUG("Device left (vid: %x, pid: %x)",
      		      open_device_desc.idVendor, open_device_desc.idProduct);
          }
        }
      }
    }
    libusb_handle_events_completed(ctx->usb_ctx, &ctx->kill_handler_thread);
  }
  return NULL;
}

/** @brief Initializes the UVC context
 * @ingroup init
 *
 * @note If you provide your own USB context, you must handle
 * libusb event processing using a function such as libusb_handle_events.
 *
 * @param[out] pctx The location where the context reference should be stored.
 * @param[in]  usb_ctx Optional USB context to use
 * @return Error opening context or UVC_SUCCESS
 */
uvc_error_t uvc_init(uvc_context_t **pctx, struct libusb_context *usb_ctx) {
  uvc_error_t ret = UVC_SUCCESS;
  uvc_context_t *ctx = calloc(1, sizeof(*ctx));

  if (usb_ctx == NULL) {
    ret = libusb_init(&ctx->usb_ctx);
    ctx->own_usb_ctx = 1;
    if (ret != UVC_SUCCESS) {
      free(ctx);
      ctx = NULL;
    }
  } else {
    ctx->own_usb_ctx = 0;
    ctx->usb_ctx = usb_ctx;
  }

  if (ctx != NULL)
    *pctx = ctx;

  if (ctx != NULL) {
    if (ctx->usb_ctx != NULL) {
      libusb_hotplug_callback_handle callback_handle;
      libusb_hotplug_register_callback(
          ctx->usb_ctx,
          LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
          0,
          LIBUSB_HOTPLUG_MATCH_ANY,
          LIBUSB_HOTPLUG_MATCH_ANY,
          LIBUSB_HOTPLUG_MATCH_ANY,
          _uvc_hotplug_callback,
          ctx,
          &ctx->hotplug_callback_handle);
    }
    /* Init lock here */
    pthread_mutex_init(&(ctx->lock), NULL);

    ctx->queue = uvc_usb_event_queue_create();
  }

  return ret;
}

/**
 * @brief Closes the UVC context, shutting down any active cameras.
 * @ingroup init
 *
 * @note This function invalides any existing references to the context's
 * cameras.
 *
 * If no USB context was provided to #uvc_init, the UVC-specific USB
 * context will be destroyed.
 *
 * @param ctx UVC context to shut down
 */
void uvc_exit(uvc_context_t *ctx) {
  uvc_device_handle_t *devh;

  DL_FOREACH(ctx->open_devices, devh) {
    uvc_close(ctx, devh);
  }

  if (ctx->own_usb_ctx) {
    libusb_hotplug_deregister_callback(ctx->usb_ctx,
                                       ctx->hotplug_callback_handle);

    ctx->kill_handler_thread = 1;
    pthread_join(ctx->handler_thread, NULL);

    libusb_exit(ctx->usb_ctx);
  }

  pthread_mutex_destroy(&(ctx->lock));

  uvc_usb_event_queue_destroy(ctx->queue);

  free(ctx);
}

/**
 * @internal
 * @brief Spawns a handler thread for the context
 * @ingroup init
 *
 * This should be called at the end of a successful uvc_open if no devices
 * are already open (and being handled).
 */
void uvc_start_handler_thread(uvc_context_t *ctx) {
  if (ctx->own_usb_ctx) {
    ctx->kill_handler_thread = 0;
    pthread_create(&ctx->handler_thread, NULL, _uvc_handle_events, (void*) ctx);
  }
}

