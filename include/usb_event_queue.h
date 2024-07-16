#ifndef LIBUVC_USB_EVENT_QUEUE_H_
#define LIBUVC_USB_EVENT_QUEUE_H_

struct uvc_usb_event {
  int vid;
  int pid;
  int type;
};
typedef struct uvc_usb_event uvc_usb_event_t;

struct uvc_usb_event_queue;
typedef struct uvc_usb_event_queue uvc_usb_event_queue_t;

uvc_usb_event_queue_t *uvc_usb_event_queue_create();
void uvc_usb_event_queue_destroy(uvc_usb_event_queue_t *queue);

int uvc_usb_event_queue_empty(uvc_usb_event_queue_t *queue);

int uvc_usb_event_queue_isfull(uvc_usb_event_queue_t *queue);

void uvc_usb_event_queue_enqueue(uvc_usb_event_queue_t *queue,
				 uvc_usb_event_t event);

uvc_usb_event_t uvc_usb_event_queue_dequeue(uvc_usb_event_queue_t *queue);

#endif  // LIBUVC_USB_EVENT_QUEUE_H_
