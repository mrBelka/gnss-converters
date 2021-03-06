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

typedef struct nmea_state_entry {
  uint32_t last_tow;
  int rate;
} nmea_state_entry_t;

typedef union sbp2nmea_msg {
  uint8_t begin;
  msg_gps_time_t sbp_gps_time;
  msg_utc_time_t sbp_utc_time;
  msg_pos_llh_t sbp_pos_llh;
  msg_vel_ned_t sbp_vel_ned;
  msg_dops_t sbp_dops;
  msg_age_corrections_t sbp_age_corr;
  msg_baseline_heading_t sbp_heading;
} sbp2nmea_msg_t;

typedef struct sbp_state_entry {
  sbp2nmea_msg_t msg;
} sbp_state_entry_t;

typedef struct sbp2nmea_state {
  uint8_t num_obs;
  uint8_t obs_seq_count;
  uint8_t obs_seq_total;
  sbp_gnss_signal_t nav_sids[MAX_SATS];
  sbp_gps_time_t obs_time;

  uint16_t base_sender_id;

  nmea_state_entry_t nmea_state[SBP2NMEA_NMEA_CNT];
  sbp_state_entry_t sbp_state[SBP2NMEA_SBP_CNT];

  float soln_freq;

  void (*cb_sbp_to_nmea)();
} sbp2nmea_t;

#ifdef __cplusplus
extern "C" {
#endif

void sbp2nmea_init(sbp2nmea_t *state, void (*cb_sbp_to_nmea)(u8 msg_id[]));

void sbp2nmea(sbp2nmea_t *state, const void *sbp_msg, sbp2nmea_sbp_id_t sbp_id);
void sbp2nmea_obs(sbp2nmea_t *state, const msg_obs_t *sbp_obs, uint8_t num_obs);

void sbp2nmea_base_id_set(sbp2nmea_t *state, uint16_t base_sender_id);
uint16_t sbp2nmea_base_id_get(const sbp2nmea_t *state);

uint8_t sbp2nmea_num_obs_get(const sbp2nmea_t *state);
const sbp_gnss_signal_t *sbp2nmea_nav_sids_get(const sbp2nmea_t *state);

void sbp2nmea_to_str(const sbp2nmea_t *state, char *sentence);

void *sbp2nmea_msg_get(const sbp2nmea_t *state, sbp2nmea_sbp_id_t id);

void sbp2nmea_rate_set(sbp2nmea_t *state, int rate, sbp2nmea_nmea_id_t id);

void sbp2nmea_soln_freq_set(sbp2nmea_t *state, float soln_freq);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_SBP_NMEA_INTERFACE_H */
