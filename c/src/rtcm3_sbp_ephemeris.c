

#include "rtcm3_sbp_internal.h"
#include <math.h>

#define power2_55 2.775557561562891e-17 /* 2^-55 */
#define power2_43 1.136868377216160e-13 /* 2^-43 */
#define power2_40 9.094947017729282e-13 /* 2^-40 */
#define power2_33 1.164153218269348e-10 /* 2^-33 */
#define power2_31 4.656612873077393e-10 /* 2^-31 */
#define power2_30 9.313225746154785e-10 /* 2^-30 */
#define power2_29 1.862645149230957e-09 /* 2^-29 */
#define power2_20 9.536743164062500e-07 /* 2^-20 */
#define power2_19 1.907348632812500e-06 /* 2^-19 */
#define power2_11 4.882812500000000e-04 /* 2^-11 */
#define power2_5 3.125000000000000e-02 /* 2^5) */

float convert_ura_to_uri(uint8_t ura) {
  /* Convert between RTCM/GPS URA ("User Range Accuracy") index to a number in meters.
   * See section 2.5.3, "User Range Accuracy", in the GPS signal specification.
   * Indices 1, 3, and 5 are hard-coded according to spec, and 15 is hard-coded according
   * to SBP/Piksi convention. */
  if(ura == 1) {
    return 2.8;
  } else if (ura == 3) {
     return 5.7;
  } else if (ura == 5) {
     return 11.3;
  } else if (ura <= 6) {
    return pow(2,(1 + (ura/2)));
  } else if (ura > 6 && ura < 15) {
    return pow(2,(ura - 2));
  } else if (ura == 15) {
    return 6144;
  }
  return -1;
}

/** Calculate the GPS ephemeris curve fit interval.
*
* \param fit_interval_flag The curve fit interval flag. 0 is 4 hours, 1 is >4
* hours.
* \param iodc The IODC value.
* \return the curve fit interval in seconds.
*/
u32 decode_fit_interval(u8 fit_interval_flag, u16 iodc) {
  u8 fit_interval = 4; /* This is in hours */

  if (fit_interval_flag) {
    fit_interval = 6;

    if ((iodc >= 240) && (iodc <= 247)) {
      fit_interval = 8;
    } else if (((iodc >= 248) && (iodc <= 255)) || (iodc == 496)) {
      fit_interval = 14;
    } else if (((iodc >= 497) && (iodc <= 503)) ||
               ((iodc >= 1021) && (iodc <= 1023))) {
      fit_interval = 26;
    } else if ((iodc >= 504) && (iodc <= 510)) {
      fit_interval = 50;
    } else if ((iodc == 511) || ((iodc >= 752) && (iodc <= 756))) {
      fit_interval = 74;
    } else if (iodc == 757) {
      fit_interval = 98;
    }
  }

  return fit_interval * 60 * 60;
}

/** Adjust the week number of wn_raw to correctly reflect the current week
 * cycle.
 *
 * Assumes the current week number cannot be earlier than the reference WN. So
 * will return the correct WN for at most 20 years after the reference WN.
 *
 * \param wn_raw Raw week number from NAV data stream that is modulo 1024
 * \param wn_ref Reference week number that is from some point in the past
 *
 * \return The absolute week number counted from 1980
 *
 * \sa rtcm3_gps_adjust_week_cycle256
 */
u16 rtcm3_gps_adjust_week_cycle(u16 wn_raw, u16 wn_ref) {
  /* note the week numbers are unsigned so they cannot be WN_UNKNOWN */
  if (wn_raw >= wn_ref) {
    return wn_raw;
  }

  return wn_raw + 1024 * ((wn_ref + 1023 - wn_raw) / 1024);
}

void rtcm3_gps_eph_to_sbp(rtcm_msg_eph *msg_eph, msg_ephemeris_gps_t *sbp_gps_eph, struct rtcm3_sbp_state *state) {
  /* RTCM gives wn module 1024, so take the current time and mask the lower 10 bits */
  sbp_gps_eph->common.toe.wn = rtcm3_gps_adjust_week_cycle(state->time_from_rover_obs.wn,msg_eph->wn);
  sbp_gps_eph->common.toe.tow = msg_eph->toe * 16;
  sbp_gps_eph->common.sid.sat = msg_eph->sat_id;
  sbp_gps_eph->common.sid.code = CODE_GPS_L1CA;
  sbp_gps_eph->common.ura = convert_ura_to_uri(msg_eph->ura);
  sbp_gps_eph->common.fit_interval = decode_fit_interval(msg_eph->fit_interval, msg_eph->kepler.iodc);
  sbp_gps_eph->common.valid = msg_eph->kepler.iodc == msg_eph->kepler.iode;
  sbp_gps_eph->common.health_bits = msg_eph->health_bits;

  sbp_gps_eph->tgd = msg_eph->kepler.tgd_gps_s * power2_31;

  sbp_gps_eph->c_rs = msg_eph->kepler.crs * power2_5;
  sbp_gps_eph->c_rc = msg_eph->kepler.crc * power2_5;
  sbp_gps_eph->c_uc = msg_eph->kepler.cuc * power2_29;
  sbp_gps_eph->c_us = msg_eph->kepler.cus * power2_29;
  sbp_gps_eph->c_ic = msg_eph->kepler.cic * power2_29;
  sbp_gps_eph->c_is = msg_eph->kepler.cis * power2_29;

  sbp_gps_eph->dn = msg_eph->kepler.dn * power2_43 * M_PI;
  sbp_gps_eph->m0 = msg_eph->kepler.m0 * power2_31 * M_PI;
  sbp_gps_eph->ecc = msg_eph->kepler.ecc * power2_33;
  sbp_gps_eph->sqrta = msg_eph->kepler.sqrta * power2_19;
  sbp_gps_eph->omega0 = msg_eph->kepler.omega0 * power2_31 * M_PI;
  sbp_gps_eph->omegadot = msg_eph->kepler.omegadot * power2_43 * M_PI;
  sbp_gps_eph->w = msg_eph->kepler.w * power2_31 * M_PI;
  sbp_gps_eph->inc = msg_eph->kepler.inc * power2_31 * M_PI;
  sbp_gps_eph->inc_dot = msg_eph->kepler.inc_dot * power2_43 * M_PI;

  sbp_gps_eph->af0 = msg_eph->kepler.af0 * power2_31;
  sbp_gps_eph->af1 = msg_eph->kepler.af1 * power2_43;
  sbp_gps_eph->af2 = msg_eph->kepler.af2 * power2_55;

  sbp_gps_eph->iode = msg_eph->kepler.iode;
  sbp_gps_eph->iodc = msg_eph->kepler.iodc;

  sbp_gps_eph->toc.wn = (state->time_from_rover_obs.wn & 0xFC00) + msg_eph->wn;
  sbp_gps_eph->toc.tow = msg_eph->kepler.toc * 16;
}

void rtcm3_glo_eph_to_sbp(rtcm_msg_eph *msg_eph, msg_ephemeris_glo_t *sbp_glo_eph, struct rtcm3_sbp_state *state) {
  sbp_glo_eph->common.toe.wn = (state->time_from_rover_obs.wn & 0xFC00) + msg_eph->wn;
  sbp_glo_eph->common.toe.tow = msg_eph->toe;
  sbp_glo_eph->common.sid.sat = msg_eph->sat_id;
  sbp_glo_eph->common.sid.code = CODE_GLO_L1OF;
  sbp_glo_eph->common.ura = msg_eph->ura;
  sbp_glo_eph->common.fit_interval = msg_eph->fit_interval;
  sbp_glo_eph->common.valid = msg_eph->valid;
  sbp_glo_eph->common.health_bits = msg_eph->health_bits;

  sbp_glo_eph->gamma = msg_eph->glo.gamma * power2_40;
  sbp_glo_eph->tau = msg_eph->glo.tau * power2_30;
  sbp_glo_eph->d_tau = msg_eph->glo.d_tau * power2_40;

  sbp_glo_eph->pos[0] = msg_eph->glo.pos[0] * power2_11 * 1000;
  sbp_glo_eph->pos[1] = msg_eph->glo.pos[1] * power2_11 * 1000;
  sbp_glo_eph->pos[2] = msg_eph->glo.pos[2] * power2_11 * 1000;

  sbp_glo_eph->vel[0] = msg_eph->glo.vel[0] * power2_20 * 1000;
  sbp_glo_eph->vel[1] = msg_eph->glo.vel[1] * power2_20 * 1000;
  sbp_glo_eph->vel[2] = msg_eph->glo.vel[2] * power2_20 * 1000;

  sbp_glo_eph->acc[0] = msg_eph->glo.acc[0] * power2_30 * 1000;
  sbp_glo_eph->acc[1] = msg_eph->glo.acc[1] * power2_30 * 1000;
  sbp_glo_eph->acc[2] = msg_eph->glo.acc[2] * power2_30 * 1000;

  sbp_glo_eph->fcn = msg_eph->glo.fcn + 1;
  //sbp_glo_eph->iod = (msg_eph->glo.t_b * 15 * 60) % 127;
}