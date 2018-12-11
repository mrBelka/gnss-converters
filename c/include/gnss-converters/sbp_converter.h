/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_SBP_CONVERTER_INTERFACE_H
#define GNSS_CONVERTERS_SBP_CONVERTER_INTERFACE_H

#include <gnss-converters/rtcm3_sbp.h>
#include <swiftnav/fifo_byte.h>
#include <libsbp/sbp.h>

#define BUFFER_SIZE 8192

struct sbp_converter {
  struct rtcm3_out_state state;
  sbp_state_t sbp_state;
  fifo_t fifo;
  uint8_t buffer[BUFFER_SIZE];
};


#ifdef __cplusplus
extern "C" {
#endif

void sbp_converter_init(struct sbp_converter *converter);

size_t sbp_converter_write(struct sbp_converter *converter, const uint8_t *buffer, size_t length);

size_t sbp_converter_read(struct sbp_converter *converter, uint8_t *buffer, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_SBP_CONVERTER_INTERFACE_H */
