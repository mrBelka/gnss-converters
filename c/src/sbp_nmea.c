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

#include "sbp_nmea_internal.h"

#include <math.h>
#include <stdio.h>

#include <gnss-converters/nmea.h>
#include <gnss-converters/sbp_nmea.h>
#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>

static double sbp_gpsdifftime(const sbp_gps_time_t *sbp_end,
                              const sbp_gps_time_t *sbp_begin);

static bool gpgga_ready(const struct sbp_nmea_state *state) {
  return (state->sbp_utc_time.tow == state->sbp_pos_llh.tow &&
          state->sbp_utc_time.tow == state->sbp_dops.tow &&
          state->sbp_utc_time.tow == state->sbp_age_corr.tow &&
          state->sbp_utc_time.tow != state->gpgga_last_tow);
}

static bool gsa_ready(const struct sbp_nmea_state *state) {
  /* Ready to send when the DOP timestamp matches with current epoch, and there
   * is a full obs message sequence not older than the solution epoch.
   * (This enables this message also in Time Matched mode, although the list of
   * active satellites will be that of current obs and not the earlier ones that
   * the solution and DOP are based on.) */
  const sbp_gps_time_t gps_time = {.wn = state->sbp_gps_time.wn,
                                   .tow = state->sbp_gps_time.tow};
  return state->sbp_utc_time.tow == state->sbp_gps_time.tow &&
         state->sbp_utc_time.tow == state->sbp_dops.tow &&
         sbp_gpsdifftime(&gps_time, &state->obs_time) <= 0 &&
         state->obs_seq_count == state->obs_seq_total - 1 &&
         state->sbp_utc_time.tow != state->gsa_last_tow;
}

static bool gprmc_ready(const struct sbp_nmea_state *state) {
  return state->sbp_utc_time.tow == state->sbp_pos_llh.tow &&
         state->sbp_utc_time.tow == state->sbp_vel_ned.tow &&
         state->sbp_utc_time.tow != state->gprmc_last_tow;
}

static bool gpvtg_ready(const struct sbp_nmea_state *state) {
  return state->sbp_utc_time.tow == state->sbp_vel_ned.tow &&
         state->sbp_utc_time.tow == state->sbp_pos_llh.tow &&
         state->sbp_utc_time.tow != state->gpvtg_last_tow;
}

static bool gpgll_ready(const struct sbp_nmea_state *state) {
  return state->sbp_utc_time.tow == state->sbp_pos_llh.tow &&
         state->sbp_utc_time.tow != state->gpgll_last_tow;
}

static bool gpzda_ready(const struct sbp_nmea_state *state) {
  return state->sbp_utc_time.tow != state->gpzda_last_tow;
}

static bool gphdt_ready(const struct sbp_nmea_state *state) {
  return state->sbp_utc_time.tow == state->sbp_heading.tow &&
         state->sbp_utc_time.tow != state->gphdt_last_tow;
}

/* Send all the NMEA messages that have become ready to send */
static void check_nmea_send(struct sbp_nmea_state *state) {
  const u32 tow = state->sbp_utc_time.tow;
  const float freq = state->soln_freq;

  /* Send each NMEA message if all its component SBP messages have been received
   * and that time matches the send rate */
  if (gpgga_ready(state) && check_nmea_rate(state->gpgga_rate, tow, freq)) {
    send_gpgga(state);
    state->gpgga_last_tow = tow;
  }
  if (gsa_ready(state) && check_nmea_rate(state->gsa_rate, tow, freq)) {
    send_gsa(state);
    state->gsa_last_tow = tow;
  }
  if (gprmc_ready(state) && check_nmea_rate(state->gprmc_rate, tow, freq)) {
    send_gprmc(state);
    state->gprmc_last_tow = tow;
  }
  if (gpvtg_ready(state) && check_nmea_rate(state->gpvtg_rate, tow, freq)) {
    send_gpvtg(state);
    state->gpvtg_last_tow = tow;
  }
  if (gpgll_ready(state) && check_nmea_rate(state->gpgll_rate, tow, freq)) {
    send_gpgll(state);
    state->gpgll_last_tow = tow;
  }
  if (gpzda_ready(state) && check_nmea_rate(state->gpzda_rate, tow, freq)) {
    send_gpzda(state);
    state->gpzda_last_tow = tow;
  }
  if (gphdt_ready(state) && check_nmea_rate(state->gphdt_rate, tow, freq)) {
    send_gphdt(state);
    state->gphdt_last_tow = tow;
  }
}

void sbp2nmea_gps_time(const msg_gps_time_t *sbp_gps_time,
                       struct sbp_nmea_state *state) {
  state->sbp_gps_time = *sbp_gps_time;
  check_nmea_send(state);
}

void sbp2nmea_utc_time(const msg_utc_time_t *sbp_utc_time,
                       struct sbp_nmea_state *state) {
  state->sbp_utc_time = *sbp_utc_time;
  check_nmea_send(state);
}

void sbp2nmea_pos_llh(const msg_pos_llh_t *sbp_pos_llh,
                      struct sbp_nmea_state *state) {
  state->sbp_pos_llh = *sbp_pos_llh;
  check_nmea_send(state);
}

void sbp2nmea_vel_ned(const msg_vel_ned_t *sbp_vel_ned,
                      struct sbp_nmea_state *state) {
  state->sbp_vel_ned = *sbp_vel_ned;
  check_nmea_send(state);
}

void sbp2nmea_dops(const msg_dops_t *sbp_dops, struct sbp_nmea_state *state) {
  state->sbp_dops = *sbp_dops;
  check_nmea_send(state);
}

void sbp2nmea_age_corrections(const msg_age_corrections_t *sbp_age_corr,
                              struct sbp_nmea_state *state) {
  state->sbp_age_corr = *sbp_age_corr;
  check_nmea_send(state);
}

void sbp2nmea_baseline_heading(const msg_baseline_heading_t *sbp_heading,
                               struct sbp_nmea_state *state) {
  state->sbp_heading = *sbp_heading;
  check_nmea_send(state);
}

void sbp2nmea_set_base_id(const uint16_t base_sender_id,
                          struct sbp_nmea_state *state) {
  state->base_sender_id = base_sender_id;
}

void unpack_obs_header(const observation_header_t *header,
                       sbp_gps_time_t *obs_time,
                       u8 *total,
                       u8 *count) {
  *obs_time = header->t;
  *total = (header->n_obs >> MSG_OBS_HEADER_SEQ_SHIFT);
  *count = (header->n_obs & MSG_OBS_HEADER_SEQ_MASK);
}

static double sbp_gpsdifftime(const sbp_gps_time_t *sbp_end,
                              const sbp_gps_time_t *sbp_begin) {
  gps_time_t end;
  gps_time_t begin;
  end.wn = sbp_end->wn;
  end.tow = (double)sbp_end->tow / SECS_MS;
  begin.wn = sbp_begin->wn;
  begin.tow = (double)sbp_begin->tow / SECS_MS;
  return gpsdifftime(&end, &begin);
}

void sbp2nmea_obs(const msg_obs_t *sbp_obs,
                  uint8_t num_obs,
                  struct sbp_nmea_state *state) {
  (void)sbp_obs;
  (void)state;

  uint8_t count;
  sbp_gps_time_t obs_time;
  unpack_obs_header(&sbp_obs->header, &obs_time, &state->obs_seq_total, &count);

  /* Count zero means it's the first message in a sequence */
  if (count == 0) {
    state->num_obs = 0;
  } else if ((fabs(sbp_gpsdifftime(&obs_time, &state->obs_time)) >
              FLOAT_EQUALITY_EPS) ||
             (state->obs_time.wn != obs_time.wn) ||
             ((state->obs_seq_count + 1) != count)) {
    /* Obs message missed, reset sequence */
    return;
  }
  state->obs_seq_count = count;
  state->obs_time = obs_time;

  for (int i = 0; i < num_obs; i++) {
    if (!(sbp_obs->obs[i].flags & OBSERVATION_VALID)) {
      state->nav_sids[state->num_obs] = sbp_obs->obs[i].sid;
      state->num_obs++;
    }
  }

  check_nmea_send(state);
}

void sbp2nmea_set_gpgga_rate(const int gpgga_rate,
                             struct sbp_nmea_state *state) {
  state->gpgga_rate = gpgga_rate;
}

void sbp2nmea_set_gprmc_rate(const int gprmc_rate,
                             struct sbp_nmea_state *state) {
  state->gprmc_rate = gprmc_rate;
}

void sbp2nmea_set_gpvtg_rate(const int gpvtg_rate,
                             struct sbp_nmea_state *state) {
  state->gpvtg_rate = gpvtg_rate;
}

void sbp2nmea_set_gphdt_rate(const int gphdt_rate,
                             struct sbp_nmea_state *state) {
  state->gphdt_rate = gphdt_rate;
}

void sbp2nmea_set_gpgll_rate(const int gpgll_rate,
                             struct sbp_nmea_state *state) {
  state->gpgll_rate = gpgll_rate;
}

void sbp2nmea_set_gpzda_rate(const int gpzda_rate,
                             struct sbp_nmea_state *state) {
  state->gpzda_rate = gpzda_rate;
}

void sbp2nmea_set_gsa_rate(const int gsa_rate, struct sbp_nmea_state *state) {
  state->gsa_rate = gsa_rate;
}

void sbp2nmea_set_soln_freq(const float soln_freq,
                            struct sbp_nmea_state *state) {
  state->soln_freq = soln_freq;
}

void sbp2nmea_init(struct sbp_nmea_state *state,
                   void (*cb_sbp_to_nmea)(u8 msg_id[])) {
  state->base_sender_id = 0;

  state->gpgga_last_tow = TOW_INVALID;
  state->gprmc_last_tow = TOW_INVALID;
  state->gpvtg_last_tow = TOW_INVALID;
  state->gphdt_last_tow = TOW_INVALID;
  state->gpgll_last_tow = TOW_INVALID;
  state->gpzda_last_tow = TOW_INVALID;
  state->gsa_last_tow = TOW_INVALID;

  state->num_obs = 0;
  state->obs_seq_count = 0;
  state->obs_seq_total = 0;
  state->obs_time.wn = 0;
  state->obs_time.tow = 0;

  state->gpgga_rate = 0;
  state->gprmc_rate = 0;
  state->gpvtg_rate = 0;
  state->gphdt_rate = 0;
  state->gpgll_rate = 0;
  state->gpzda_rate = 0;
  state->gsa_rate = 0;

  state->soln_freq = 0.0;

  state->cb_sbp_to_nmea = cb_sbp_to_nmea;
}
