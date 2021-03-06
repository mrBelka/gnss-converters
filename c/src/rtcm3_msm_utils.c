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

/* To illustrate the relationship between PRN, MSM satellite ID, satellite mask,
 * and the 0-based satellite index, used to index the satellite data array,
 * take for example QZS with measurements from PRNs 193, 194 and 196:
 *
 *                       PRN  |  193 |  194 |   193 |  196 |   197 |   198 | ...
 * MSM sat ID (Table 3.5-104) |    1 |    2 |     3 |    4 |     5 |     6 | ...
 *            satellite mask  | true | true | false | true | false | false | ...
 *            satellite index |    0 |    1 |       |    2 |       |       |
 *
 */

#include "rtcm3_msm_utils.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include <rtcm3/constants.h>
#include <rtcm3/msm_utils.h>
#include <swiftnav/constants.h>
#include <swiftnav/signal.h>

/** Define the PRN ranges for each constellation. */
typedef struct {
  u16 first_prn;
  u16 num_sats;
} prn_table_element_t;

static const prn_table_element_t prn_table[RTCM_CONSTELLATION_COUNT] = {
        [RTCM_CONSTELLATION_GPS] = {GPS_FIRST_PRN, NUM_SATS_GPS},
        [RTCM_CONSTELLATION_SBAS] = {SBAS_FIRST_PRN, NUM_SATS_SBAS},
        [RTCM_CONSTELLATION_GLO] = {GLO_FIRST_PRN, NUM_SATS_GLO},
        [RTCM_CONSTELLATION_BDS] = {BDS_FIRST_PRN, NUM_SATS_BDS},
        [RTCM_CONSTELLATION_QZS] = {QZS_FIRST_PRN, NUM_SATS_QZS},
        [RTCM_CONSTELLATION_GAL] = {GAL_FIRST_PRN, NUM_SATS_GAL},
};

static code_t get_msm_gps_code(u8 signal_id) {
  /* RTCM 10403.3 Table 3.5-91 */
  switch (signal_id) {
    case 2: /* 1C */
      return CODE_GPS_L1CA;
    case 3: /* 1P */
      return CODE_GPS_L1P;
    case 4: /* 1W */
      return CODE_GPS_L1P;
    /* case 8: 2C */
    case 9: /* 2P */
      return CODE_GPS_L2P;
    case 10: /* 2W */
      return CODE_GPS_L2P;
    case 15: /* 2S */
      return CODE_GPS_L2CM;
    case 16: /* 2L */
      return CODE_GPS_L2CL;
    case 17: /* 2X */
      return CODE_GPS_L2CX;
    case 22: /* 5I */
      return CODE_GPS_L5I;
    case 23: /* 5Q */
      return CODE_GPS_L5Q;
    case 24: /* 5X */
      return CODE_GPS_L5X;
    case 30: /* 1S */
      return CODE_GPS_L1CI;
    case 31: /* 1L */
      return CODE_GPS_L1CQ;
    case 32: /* 1X */
      return CODE_GPS_L1CX;
    default:
      return CODE_INVALID;
  }
}

static code_t get_msm_glo_code(u8 signal_id) {
  /* RTCM 10403.3 Table 3.5-96 */
  switch (signal_id) {
    case 3: /* 1P */
      return CODE_GLO_L1P;
    case 2: /* 1C */
      return CODE_GLO_L1OF;
    case 9: /* 2P */
      return CODE_GLO_L2P;
    case 8: /* 2C */
      return CODE_GLO_L2OF;
    default:
      return CODE_INVALID;
  }
}

static code_t get_msm_gal_code(u8 signal_id) {
  /* RTCM 10403.3 Table 3.5-99 */
  switch (signal_id) {
    case 2: /* 1C */
      return CODE_GAL_E1C;
    /* case 3: 1A */
    case 4: /* 1B */
      return CODE_GAL_E1B;
    case 5: /* 1X */
      return CODE_GAL_E1X;
    /* case 6: 1Z */
    case 8: /* 6C */
      return CODE_GAL_E6C;
    /* case 9: 6A */
    case 10: /* 6B */
      return CODE_GAL_E6B;
    case 11: /* 6X */
      return CODE_GAL_E6X;
    /* case 12: 6Z */
    case 14: /* 7I */
      return CODE_GAL_E7I;
    case 15: /* 7Q */
      return CODE_GAL_E7Q;
    case 16: /* 7X */
      return CODE_GAL_E7X;
    case 18: /* 8I */
      return CODE_GAL_E8I;
    case 19: /* 8Q */
      return CODE_GAL_E8Q;
    case 20: /* 8X */
      return CODE_GAL_E8X;
    case 22: /* 5I */
      return CODE_GAL_E5I;
    case 23: /* 5Q */
      return CODE_GAL_E5Q;
    case 24: /* 5X */
      return CODE_GAL_E5X;
    default:
      return CODE_INVALID;
  }
}

static code_t get_msm_sbas_code(u8 signal_id) {
  /* RTCM 10403.3 Table 3.5-102 */
  switch (signal_id) {
    case 2: /* 1C */
      return CODE_SBAS_L1CA;
    case 22: /* 5I */
      return CODE_SBAS_L5I;
    case 23: /* 5Q */
      return CODE_SBAS_L5Q;
    case 24: /* 5X */
      return CODE_SBAS_L5X;
    default:
      return CODE_INVALID;
  }
}

static code_t get_msm_qzs_code(u8 signal_id) {
  /* RTCM 10403.3 Table 3.5-105 */
  switch (signal_id) {
    case 2: /* 1C */
      return CODE_QZS_L1CA;
    /* case 9:  6S */
    /* case 10: 6L */
    /* case 11: 6X */
    case 15: /* 2S */
      return CODE_QZS_L2CM;
    case 16: /* 2L */
      return CODE_QZS_L2CL;
    case 17: /* 2X */
      return CODE_QZS_L2CX;
    case 22: /* 5I */
      return CODE_QZS_L5I;
    case 23: /* 5Q */
      return CODE_QZS_L5Q;
    case 24: /* 5X */
      return CODE_QZS_L5X;
    case 30: /* 1S */
      return CODE_QZS_L1CI;
    case 31: /* 1L */
      return CODE_QZS_L1CQ;
    case 32: /* 1X */
      return CODE_QZS_L1CX;
    default:
      return CODE_INVALID;
  }
}

static code_t get_msm_bds2_code(u8 signal_id) {
  /* RTCM 10403.3 Table 3.5-108 */
  switch (signal_id) {
    case 2: /* 2I */
      return CODE_BDS2_B1;
    /* case 3:  2Q */
    /* case 4:  2X */
    /* case 8:  6I */
    /* case 9:  6Q */
    /* case 10:  6X */
    case 14: /* 7I */
      return CODE_BDS2_B2;
    /* case 15:  7Q */
    /* case 16:  7X */
    default:
      return CODE_INVALID;
  }
}

/** Get the code enum of an MSM signal
 *
 * \param header Pointer to message header
 * \param signal_index 0-based index into the signal mask
 * \return code enum (CODE_INVALID for unsupported codes/constellations)
 */
code_t msm_signal_to_code(const rtcm_msm_header *header, u8 signal_index) {
  rtcm_constellation_t cons = to_constellation(header->msg_num);
  assert(signal_index <= MSM_SIGNAL_MASK_SIZE);
  u8 code_index =
      find_nth_mask_value(
          MSM_SIGNAL_MASK_SIZE, header->signal_mask, signal_index + 1) +
      1;

  switch (cons) {
    case RTCM_CONSTELLATION_GPS:
      return get_msm_gps_code(code_index);
    case RTCM_CONSTELLATION_SBAS:
      return get_msm_sbas_code(code_index);
    case RTCM_CONSTELLATION_GLO:
      return get_msm_glo_code(code_index);
    case RTCM_CONSTELLATION_BDS:
      return get_msm_bds2_code(code_index);
    case RTCM_CONSTELLATION_QZS:
      return get_msm_qzs_code(code_index);
    case RTCM_CONSTELLATION_GAL:
      return get_msm_gal_code(code_index);
    case RTCM_CONSTELLATION_INVALID:
    case RTCM_CONSTELLATION_COUNT:
    default:
      return CODE_INVALID;
  }
}

/** Get the index to MSM signal mask corresponding to a code enum
 *
 * \param header Pointer to message header
 * \param code code enum
 * \return signal_index 0-based index into the signal mask
 */
u8 code_to_msm_signal_index(const rtcm_msm_header *header, code_t code) {
  rtcm_constellation_t cons = to_constellation(header->msg_num);
  u8 signal_id = code_to_msm_signal_id(code, cons);
  assert(signal_id <= MSM_SIGNAL_MASK_SIZE);
  return count_mask_values(signal_id, header->signal_mask);
}

/** Get the MSM signal id from code enum
 *
 * \param code enum
 * \param cons constellation enum
 * \return The 0-based signal_id
 */
u8 code_to_msm_signal_id(const code_t code, const rtcm_constellation_t cons) {
  assert(RTCM_CONSTELLATION_INVALID != cons &&
         RTCM_CONSTELLATION_COUNT != cons);
  assert(CODE_INVALID != code);
  /* look up using the signal to code functions */
  switch (cons) {
    case RTCM_CONSTELLATION_GPS:
      for (u8 code_index = 0; code_index < MSM_SIGNAL_MASK_SIZE; code_index++) {
        if (get_msm_gps_code(code_index + 1) == code) {
          return code_index;
        }
      }
      break;
    case RTCM_CONSTELLATION_SBAS:
      for (u8 code_index = 0; code_index < MSM_SIGNAL_MASK_SIZE; code_index++) {
        if (get_msm_sbas_code(code_index + 1) == code) {
          return code_index;
        }
      }
      break;
    case RTCM_CONSTELLATION_GLO:
      for (u8 code_index = 0; code_index < MSM_SIGNAL_MASK_SIZE; code_index++) {
        if (get_msm_glo_code(code_index + 1) == code) {
          return code_index;
        }
      }
      break;
    case RTCM_CONSTELLATION_BDS:
      for (u8 code_index = 0; code_index < MSM_SIGNAL_MASK_SIZE; code_index++) {
        if (get_msm_bds2_code(code_index + 1) == code) {
          return code_index;
        }
      }
      break;
    case RTCM_CONSTELLATION_QZS:
      for (u8 code_index = 0; code_index < MSM_SIGNAL_MASK_SIZE; code_index++) {
        if (get_msm_qzs_code(code_index + 1) == code) {
          return code_index;
        }
      }
      break;
    case RTCM_CONSTELLATION_GAL:
      for (u8 code_index = 0; code_index < MSM_SIGNAL_MASK_SIZE; code_index++) {
        if (get_msm_gal_code(code_index + 1) == code) {
          return code_index;
        }
      }
      break;
    case RTCM_CONSTELLATION_INVALID:
    case RTCM_CONSTELLATION_COUNT:
    default:
      break;
  }
  fprintf(stderr, "Code %d not found in RTCM constellation %u\n", code, cons);
  return MSM_SIGNAL_MASK_SIZE;
}

static bool prn_valid(rtcm_constellation_t cons, u8 prn) {
  return (RTCM_CONSTELLATION_INVALID != cons) &&
         (RTCM_CONSTELLATION_COUNT > cons) &&
         (prn >= prn_table[cons].first_prn) &&
         (prn < prn_table[cons].first_prn + prn_table[cons].num_sats);
}

/** Get the PRN from an MSM satellite index
 *
 * \param header Pointer to message header
 * \param satellite_index 0-based index into the satellite mask
 * \return PRN (or 0 for invalid constellation)
 */
u8 msm_sat_to_prn(const rtcm_msm_header *header, u8 satellite_index) {
  rtcm_constellation_t cons = to_constellation(header->msg_num);
  if (RTCM_CONSTELLATION_INVALID == cons || RTCM_CONSTELLATION_COUNT == cons) {
    return PRN_INVALID;
  }
  assert(satellite_index <= MSM_SATELLITE_MASK_SIZE);
  u8 prn_index = find_nth_mask_value(
      MSM_SATELLITE_MASK_SIZE, header->satellite_mask, satellite_index + 1);

  u8 prn = prn_table[cons].first_prn + prn_index;
  return prn_valid(cons, prn) ? prn : PRN_INVALID;
}

/** Get the index to MSM satellite mask corresponding to a PRN
 *
 * \param header Pointer to message header
 * \param PRN
 * \return satellite_index 0-based index into the satellite mask
 */
u8 prn_to_msm_sat_index(const rtcm_msm_header *header, u8 prn) {
  rtcm_constellation_t cons = to_constellation(header->msg_num);
  u8 sat_id = prn_to_msm_sat_id(prn, cons);
  assert(sat_id <= MSM_SATELLITE_MASK_SIZE);
  return count_mask_values(sat_id, header->satellite_mask);
}

/** Get the MSM satellite ID corresponding to a PRN
 *
 * \param PRN
 * \param cons constellation enum
 * \return satellite_index 0-based index into the satellite mask
 */
u8 prn_to_msm_sat_id(u8 prn, rtcm_constellation_t cons) {
  assert(prn_valid(cons, prn));
  return prn - prn_table[cons].first_prn;
}

/** Find the frequency of an MSM signal
 *
 * \param header Pointer to message header
 * \param signal_index 0-based index into the signal mask
 * \param glo_fcn The FCN value for GLO satellites
 * \param glo_fcn_valid Validity flag for glo_fcn
 * \param p_freq Pointer to write the frequency output to
 * \return true if a valid frequency was returned
 */
bool msm_signal_frequency(const rtcm_msm_header *header,
                          const u8 signal_index,
                          const u8 glo_fcn,
                          const bool glo_fcn_valid,
                          double *p_freq) {
  assert(signal_index <= MSM_SIGNAL_MASK_SIZE);
  code_t code = msm_signal_to_code(header, signal_index);

  /* TODO: use sid_to_carr_freq from LNSP */
  /* TODO: remove glo_fcn_valid parameter and use
   * glo_fcn=MSM_GLO_FCN_UNKNOWN instead */

  switch ((int8_t)code) {
    case CODE_GPS_L1CA:
    case CODE_GPS_L1P:
    case CODE_GPS_L1CI:
    case CODE_GPS_L1CQ:
    case CODE_GPS_L1CX:
      *p_freq = GPS_L1_HZ;
      return true;
    case CODE_GPS_L2CM:
    case CODE_GPS_L2CL:
    case CODE_GPS_L2CX:
    case CODE_GPS_L2P:
      *p_freq = GPS_L2_HZ;
      return true;
    case CODE_GPS_L5I:
    case CODE_GPS_L5Q:
    case CODE_GPS_L5X:
      *p_freq = GPS_L5_HZ;
      return true;
    case CODE_GLO_L1OF:
    case CODE_GLO_L1P:
      /* GLO FCN given in the sat info field, see Table 3.4-6 */
      if (glo_fcn_valid) {
        *p_freq = GLO_L1_HZ + (glo_fcn - MSM_GLO_FCN_OFFSET) * GLO_L1_DELTA_HZ;
        return true;
      } else {
        return false;
      }
    case CODE_GLO_L2OF:
    case CODE_GLO_L2P:
      if (glo_fcn_valid) {
        *p_freq = GLO_L2_HZ + (glo_fcn - MSM_GLO_FCN_OFFSET) * GLO_L2_DELTA_HZ;
        return true;
      } else {
        return false;
      }
    case CODE_BDS2_B1:
      *p_freq = BDS2_B11_HZ;
      return true;
    case CODE_BDS2_B2:
      *p_freq = BDS2_B2_HZ;
      return true;
    case CODE_SBAS_L1CA:
      *p_freq = SBAS_L1_HZ;
      return true;
    case CODE_SBAS_L5I:
    case CODE_SBAS_L5Q:
    case CODE_SBAS_L5X:
      *p_freq = SBAS_L5_HZ;
      return true;
    case CODE_GAL_E1B:
    case CODE_GAL_E1C:
    case CODE_GAL_E1X:
      *p_freq = GAL_E1_HZ;
      return true;
    case CODE_GAL_E7I:
    case CODE_GAL_E7Q:
    case CODE_GAL_E7X:
      *p_freq = GAL_E7_HZ;
      return true;
    case CODE_GAL_E5I:
    case CODE_GAL_E5Q:
    case CODE_GAL_E5X:
      *p_freq = GAL_E5_HZ;
      return true;
    case CODE_GAL_E6B:
    case CODE_GAL_E6C:
    case CODE_GAL_E6X:
      *p_freq = GAL_E6_HZ;
      return true;
    case CODE_GAL_E8I:
    case CODE_GAL_E8Q:
    case CODE_GAL_E8X:
      *p_freq = GAL_E8_HZ;
      return true;
    case CODE_QZS_L1CA:
    case CODE_QZS_L1CI:
    case CODE_QZS_L1CQ:
    case CODE_QZS_L1CX:
      *p_freq = QZS_L1_HZ;
      return true;
    case CODE_QZS_L2CM:
    case CODE_QZS_L2CL:
    case CODE_QZS_L2CX:
      *p_freq = QZS_L2_HZ;
      return true;
    case CODE_QZS_L5I:
    case CODE_QZS_L5Q:
    case CODE_QZS_L5X:
      *p_freq = QZS_L5_HZ;
      return true;
    case CODE_INVALID:
    case CODE_COUNT:
    default:
      return false;
  }
}

/** Find the frequency channel number (FCN) of a GLO signal
 *
 * \param header Pointer to message header
 * \param sat_index 0-based index into the satellite mask
 * \param fcn_from_satinfo FCN (or MSM_GLO_FCN_UNKNOWN)
 * \param glo_sv_id_fcn_map Optional GLO FCN table (size MAX_GLO_PRN + 1)
 * \param glo_fcn Output pointer for the FCN value
 * \return true if a valid FCN was returned
 */
bool msm_get_glo_fcn(const rtcm_msm_header *header,
                     const u8 sat,
                     const u8 fcn_from_sat_info,
                     const u8 glo_sv_id_fcn_map[],
                     u8 *glo_fcn) {
  if (RTCM_CONSTELLATION_GLO != to_constellation(header->msg_num)) {
    return false;
  }

  /* get FCN from sat_info if valid */
  *glo_fcn = fcn_from_sat_info;
  if (MSM_GLO_FCN_UNKNOWN == *glo_fcn && NULL != glo_sv_id_fcn_map) {
    /* use the lookup table if given */
    u8 sat_id = msm_sat_to_prn(header, sat);
    *glo_fcn = glo_sv_id_fcn_map[sat_id];
  }
  /* valid values are from 0 to MSM_GLO_MAX_FCN */
  return (*glo_fcn <= MSM_GLO_MAX_FCN);
}

/** Return the RTCM message number for a constellation and MSM message type
 * \param cons RTCM constellation enum
 * \param msm_type MSM message type enum
 * \return RTCM message number, or 0 on failure
 */
u16 to_msm_msg_num(rtcm_constellation_t cons, msm_enum msm_type) {
  if (MSM_UNKNOWN == msm_type) {
    return 0;
  }
  switch (cons) {
    case RTCM_CONSTELLATION_GPS:
      return 1070 + (u8)msm_type;
    case RTCM_CONSTELLATION_GLO:
      return 1080 + (u8)msm_type;
    case RTCM_CONSTELLATION_GAL:
      return 1090 + (u8)msm_type;
    case RTCM_CONSTELLATION_SBAS:
      return 1100 + (u8)msm_type;
    case RTCM_CONSTELLATION_QZS:
      return 1110 + (u8)msm_type;
    case RTCM_CONSTELLATION_BDS:
      return 1120 + (u8)msm_type;
    case RTCM_CONSTELLATION_INVALID:
    case RTCM_CONSTELLATION_COUNT:
    default:
      return 0;
  }
}

u8 msm_get_num_signals(const rtcm_msm_header *header) {
  return count_mask_values(MSM_SIGNAL_MASK_SIZE, header->signal_mask);
}

u8 msm_get_num_satellites(const rtcm_msm_header *header) {
  return count_mask_values(MSM_SATELLITE_MASK_SIZE, header->satellite_mask);
}

u8 msm_get_num_cells(const rtcm_msm_header *header) {
  u8 cell_size = msm_get_num_satellites(header) * msm_get_num_signals(header);
  return count_mask_values(cell_size, header->cell_mask);
}

static void msm_add_to_header_err(code_t code, u8 prn, const char reason[]) {
  char code_string[SID_STR_LEN_MAX];
  gnss_signal_t sid = {prn, code};
  sid_to_string(code_string, SID_STR_LEN_MAX, sid);
  fprintf(stderr, "Cannot add %s to MSM header: %s\n", code_string, reason);
}

/* add the given signal and satellite to the MSM header, returns true if
 * successful, false on failure (invalid signal, or cell mask full) */
bool msm_add_to_header(rtcm_msm_header *header, code_t code, u8 prn) {
  /* should not try to add more satellites or signals if cell mask is already
   * filled */
  assert(msm_get_num_cells(header) == 0);

  rtcm_constellation_t cons = to_constellation(header->msg_num);

  if (CODE_INVALID == code) {
    msm_add_to_header_err(code, prn, "invalid code");
    return false;
  }
  if (!prn_valid(cons, prn)) {
    msm_add_to_header_err(code, prn, "invalid PRN");
    return false;
  }

  u8 sat_id = prn_to_msm_sat_id(prn, cons);
  u8 signal_id = code_to_msm_signal_id(code, cons);

  u8 num_sats = msm_get_num_satellites(header);
  u8 num_signals = msm_get_num_signals(header);

  /* add a new satellite to the mask only if it fits in the cell mask */
  if (!header->satellite_mask[sat_id]) {
    if ((num_sats + 1) * num_signals <= MSM_MAX_CELLS) {
      header->satellite_mask[sat_id] = true;
      num_sats++;
    } else {
      msm_add_to_header_err(
          code, prn, "cell size limit for satellites reached");
      return false;
    }
  }

  /* add a new signal to the mask only if it fits in the cell mask */
  if (!header->signal_mask[signal_id]) {
    if (num_sats * (num_signals + 1) <= MSM_MAX_CELLS) {
      header->signal_mask[signal_id] = true;
      num_signals++;
    } else {
      msm_add_to_header_err(code, prn, "cell size limit for signals reached");
      return false;
    }
  }
  return true;
}

/* given a header with satellite and signal masks built, allocate a cell for the
 * given signal and satellite */
bool msm_add_to_cell_mask(rtcm_msm_header *header, code_t code, u8 prn) {
  uint8_t num_sigs = msm_get_num_signals(header);
  assert(num_sigs > 0);

  rtcm_constellation_t cons = to_constellation(header->msg_num);

  if (CODE_INVALID == code) {
    msm_add_to_header_err(code, prn, "invalid code");
    return false;
  }
  if (!prn_valid(cons, prn)) {
    msm_add_to_header_err(code, prn, "invalid PRN");
    return false;
  }

  u8 sat_id = prn_to_msm_sat_id(prn, cons);
  if (!header->satellite_mask[sat_id]) {
    msm_add_to_header_err(code, prn, "not in satellite mask");
    return false;
  }

  u8 signal_id = code_to_msm_signal_id(code, cons);
  if (!header->signal_mask[signal_id]) {
    msm_add_to_header_err(code, prn, "not in signal mask");
    return false;
  }

  u8 sat_index = prn_to_msm_sat_index(header, prn);
  u8 signal_index = code_to_msm_signal_index(header, code);

  /* Mark the cell defined by this satellite/signal pair in the cell mask */
  u8 cell_id = sat_index * num_sigs + signal_index;
  assert(cell_id < MSM_MAX_CELLS);
  assert(!header->cell_mask[cell_id]);
  header->cell_mask[cell_id] = true;

  return true;
}
