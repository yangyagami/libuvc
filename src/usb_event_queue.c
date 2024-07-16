#include "usb_event_queue.h"

#include <assert.h>
#include <memory.h>
#include <pthread.h>
#include <stdlib.h>

#define UVC_QUEUE_MAX_SIZE 256

struct uvc_usb_event_queue {
  pthread_mutex_t lock;
  uvc_usb_event_t buffer[UVC_QUEUE_MAX_SIZE];
  int front;
  int back;
};

uvc_usb_event_queue_t *uvc_usb_event_queue_create() {
  uvc_usb_event_queue_t *queue;
  queue = calloc(1, sizeof(*queue));

  pthread_mutex_init(&(queue->lock), NULL);

  return queue;
}

void uvc_usb_event_queue_destroy(uvc_usb_event_queue_t *queue) {
  assert(queue != NULL);
  pthread_mutex_destroy(&(queue->lock));
  free(queue);
}

int uvc_usb_event_queue_empty(uvc_usb_event_queue_t *queue) {
  int ret = 0;
  pthread_mutex_lock(&(queue->lock));

  ret = queue->back == queue->front;

  pthread_mutex_unlock(&(queue->lock));

  return ret;
}

int uvc_usb_event_queue_isfull(uvc_usb_event_queue_t *queue) {
  int ret = 0;
  pthread_mutex_lock(&(queue->lock));

  ret = ((queue->back + 1) % UVC_QUEUE_MAX_SIZE) == queue->front;

  pthread_mutex_unlock(&(queue->lock));

  return ret;
}

void uvc_usb_event_queue_enqueue(uvc_usb_event_queue_t *queue, uvc_usb_event_t event) {
  pthread_mutex_lock(&(queue->lock));

  assert(!uvc_usb_event_queue_isfull(queue));

  queue->buffer[queue->back] = event;
  queue->back = (queue->back + 1) % UVC_QUEUE_MAX_SIZE;

  pthread_mutex_unlock(&(queue->lock));
}

uvc_usb_event_t uvc_usb_event_queue_dequeue(uvc_usb_event_queue_t *queue) {
  pthread_mutex_lock(&(queue->lock));

  assert(!uvc_usb_event_queue_empty(queue));

  uvc_usb_event_t event;
  event = queue->buffer[queue->front];
  queue->front = (queue->front + 1) % UVC_QUEUE_MAX_SIZE;

  pthread_mutex_unlock(&(queue->lock));

  return event;
}

#undef UVC_QUEUE_MAX_SIZE
