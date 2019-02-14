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

#ifndef GNSS_CONVERTERS_SBP_NMEA_INTERFACE_H
#define GNSS_CONVERTERS_SBP_NMEA_INTERFACE_H

#include <libsbp/gnss.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/orientation.h>

/* Max number of sats visible in an epoch */
#define MAX_SATS 256

typedef enum sbp2nmea_nmea_id {
  SBP2NMEA_NMEA_GGA = 0,
  SBP2NMEA_NMEA_RMC = 1,
  SBP2NMEA_NMEA_VTG = 2,
  SBP2NMEA_NMEA_HDT = 3,
  SBP2NMEA_NMEA_GLL = 4,
  SBP2NMEA_NMEA_ZDA = 5,
  SBP2NMEA_NMEA_GSA = 6,
  SBP2NMEA_NMEA_CNT = 7,
} sbp2nmea_nmea_id_t;

typedef enum sbp2nmea_sbp_id {
  SBP2NMEA_SBP_GPS_TIME = 0,
  SBP2NMEA_SBP_UTC_TIME = 1,
  SBP2NMEA_SBP_POS_LLH = 2,
  SBP2NMEA_SBP_VEL_NED = 3,
  SBP2NMEA_SBP_DOPS = 4,
  SBP2NMEA_SBP_AGE_CORR = 5,
  SBP2NMEA_SBP_HDG = 6,
  SBP2NMEA_SBP_CNT = 7,
} sbp2nmea_sbp_id_t;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sbp_nmea_state sbp2nmea_t;

sbp2nmea_t *sbp2nmea_init(void (*cb_sbp_to_nmea)(u8 msg_id[]));
void sbp2nmea_destroy(sbp2nmea_t **state);

void sbp2nmea(sbp2nmea_t *state, const void *sbp_msg, sbp2nmea_sbp_id_t id);
void sbp2nmea_obs(sbp2nmea_t *state, const msg_obs_t *sbp_obs, uint8_t num_obs);

void sbp2nmea_base_id_set(sbp2nmea_t *state, uint16_t base_sender_id);
uint16_t sbp2nmea_base_id_get(const sbp2nmea_t *state);

uint8_t sbp2nmea_num_obs_get(const sbp2nmea_t *state);
const sbp_gnss_signal_t *sbp2nmea_nav_sids_get(const sbp2nmea_t *state);

void sbp2nmea_to_str(const sbp2nmea_t *state, char *sentence);

void *sbp2nmea_message_get(const sbp2nmea_t *state, sbp2nmea_sbp_id_t id);

void sbp2nmea_rate_set(sbp2nmea_t *state, int rate, sbp2nmea_nmea_id_t id);

void sbp2nmea_soln_freq_set(sbp2nmea_t *state, float soln_freq);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_SBP_NMEA_INTERFACE_H */
