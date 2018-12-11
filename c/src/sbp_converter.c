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

static void sbp_converter_callback(uint8_t *buffer, uint16_t n, void *context) {
  fifo_t *fifo = context;
  assert(fifo != NULL);
  fifo_write(fifo, buffer, n);
}

void sbp_converter_init(struct sbp_converter *converter) {
  gps_time_t gps_time = time2gps_t(time(NULL));
  double gps_utc_offset = get_gps_utc_offset(&gps_time, NULL);
  sbp2rtcm_init(&converter->state, sbp_converter_callback, &converter->fifo);
  sbp2rtcm_set_leap_second(gps_utc_offset, &converter->state);

  fifo_init(&converter->fifo, converter->buffer, sizeof(converter->buffer));

  sbp_state_init(&converter->sbp_state);
  sbp_register_callback(&converter->sbp_state,
                        SBP_MSG_BASE_POS_ECEF,
                        (void *)&sbp2rtcm_base_pos_ecef_cb,
                        &converter->state,
                        &converter->base_pos_ecef_callback_node);
  sbp_register_callback(&converter->sbp_state,
                        SBP_MSG_OBS,
                        (void *)&sbp2rtcm_sbp_obs_cb,
                        &converter->state,
                        &converter->obs_callback_node);
}

ssize_t sbp_converter_write(struct sbp_converter *converter, uint16_t sender, uint16_t type, uint8_t *buffer, size_t length) {
  return sbp_process_payload(&converter->sbp_state, sender, type, length, buffer);
}

size_t sbp_converter_read(struct sbp_converter *converter, uint8_t *buffer, size_t length) {
  return fifo_read(&converter->fifo, buffer, length);
}
