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

#include <time.h>
#include <assert.h>
#include <gnss-converters/sbp_converter.h>
#include <swiftnav/gnss_time.h>
#include <stdlib.h>

static void sbp_converter_callback(uint8_t *buf, uint16_t len, void *context) {
  fifo_t *fifo = context;
  assert(fifo != NULL);
  fifo_write(fifo, buf, len);
}

sbp_converter_t *sbp_converter_new() {
  sbp_converter_t *converter = malloc(sizeof(sbp_converter_t));
  if (converter != NULL) {
    fifo_init(&converter->fifo, converter->buf, sizeof(converter->buf));
    sbp2rtcm_init(&converter->state, sbp_converter_callback, &converter->fifo);
    gps_time_t gps_time = time2gps_t(time(NULL));
    sbp2rtcm_set_leap_second(get_gps_utc_offset(&gps_time, NULL), &converter->state);
  }
  return converter;
}

void sbp_converter_delete(sbp_converter_t *converter) {
  free(converter);
}

size_t sbp_converter_convert(sbp_converter_t *converter, uint16_t sender, uint16_t type,
                             uint8_t *rbuf, size_t rlen, uint8_t *wbuf, size_t wlen) {
  switch (type) {
  case SBP_MSG_BASE_POS_ECEF: {
    sbp2rtcm_base_pos_ecef_cb(sender, rlen, rbuf, &converter->state);
    break;
  }
  case SBP_MSG_OBS: {
    sbp2rtcm_sbp_obs_cb(sender, rlen, rbuf, &converter->state);
    break;
  }
  }
  return fifo_read(&converter->fifo, wbuf, wlen);
}
