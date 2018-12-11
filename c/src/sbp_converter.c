/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <gnss-converters/sbp_converter.h>

static void sbp_converter_callback(uint8_t *buffer, uint16_t n, void *context) {
  (void)buffer;
  (void)n;
  fifo_t *fifo = context;
  assert(fifo != NULL);
}

void sbp_converter_init(struct sbp_converter *converter) {
  sbp2rtcm_init(&converter->state, sbp_converter_callback, &converter->fifo);
  sbp2rtcm_set_leap_second(18, &converter->state); /* TODO */
  sbp_state_init(&converter->sbp_state);
}

size_t sbp_converter_write(struct sbp_converter *converter, const uint8_t *buf, size_t length) {
  (void)converter;
  (void)buf;
  (void)length;
  return 0;
}

size_t sbp_converter_read(struct sbp_converter *converter, uint8_t *buf, size_t length) {
  (void)converter;
  (void)buf;
  (void)length;
  return 0;
}
