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

typedef struct sbp_converter_s {
  struct rtcm3_out_state state;
  fifo_t fifo;
  uint8_t buf[8192];
} sbp_converter_t;

#ifdef __cplusplus
extern "C" {
#endif

sbp_converter_t *sbp_converter_new();

void sbp_converter_delete(sbp_converter_t *converter);

size_t sbp_converter_convert(sbp_converter_t *converter, uint16_t sender, uint16_t type,
                             uint8_t *rbuf, size_t rlen, uint8_t *wbuf, size_t wlen);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_SBP_CONVERTER_INTERFACE_H */
