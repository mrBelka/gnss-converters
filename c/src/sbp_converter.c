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

static void sbp_converter_callback(uint8_t *buffer, uint16_t length, void *context) {
  fifo_t *fifo = context;
  assert(fifo != NULL);
  fifo_write(fifo, buffer, length);
}

void sbp_converter_init(struct rtcm3_out_state *state, fifo_t *fifo, uint8_t *buffer, size_t length) {
  fifo_init(fifo, buffer, length);
  sbp2rtcm_init(state, sbp_converter_callback, fifo);
  gps_time_t gps_time = time2gps_t(time(NULL));
  double gps_utc_offset = get_gps_utc_offset(&gps_time, NULL);
  sbp2rtcm_set_leap_second(gps_utc_offset, state);
}

size_t sbp_converter_write(struct rtcm3_out_state *state, fifo_t *fifo, uint16_t sender, uint16_t type,
                           uint8_t *rbuf, size_t rlen, uint8_t *wbuf, size_t wlen) {
  switch (type) {
  case SBP_MSG_BASE_POS_ECEF: {
    sbp2rtcm_base_pos_ecef_cb(sender, rlen, rbuf, state);
    break;
  }
  case SBP_MSG_OBS: {
    sbp2rtcm_sbp_obs_cb(sender, rlen, rbuf, state);
    break;
  }
  }

  return fifo_read(fifo, wbuf, wlen);
}
