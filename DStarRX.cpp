/*
 *   Copyright (C) 2009-2017,2020 by Jonathan Naylor G4KLX
 *   Copyright (C) 2017 by Andy Uribe CA6JAU
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "Config.h"

#if defined(MODE_DSTAR)

#include "Globals.h"
#include "DStarRX.h"
#include "Utils.h"

const unsigned int MAX_FRAMES = 150U;

// D-Star bit order version of 0x55 0x55 0x6E 0x0A
const uint64_t FRAME_SYNC_DATA = 0x0000000000557650U;
const uint64_t FRAME_SYNC_MASK = 0x0000000000FFFFFFU;
const uint8_t  FRAME_SYNC_ERRS = 2U;

// D-Star bit order version of 0x55 0x2D 0x16
const uint64_t DATA_SYNC_DATA = 0x0000000000AAB468U;
const uint64_t DATA_SYNC_MASK = 0x0000000000FFFFFFU;
const uint8_t  DATA_SYNC_ERRS = 2U;

// D-Star bit order version of 0x55 0x55 0xC8 0x7A
const uint64_t END_SYNC_DATA = 0x0000AAAAAAAA135EU;
const uint64_t END_SYNC_MASK = 0x0000FFFFFFFFFFFFU;
const uint8_t  END_SYNC_ERRS = 1U;

const uint8_t BIT_MASK_TABLE0[] = {0x7FU, 0xBFU, 0xDFU, 0xEFU, 0xF7U, 0xFBU, 0xFDU, 0xFEU};
const uint8_t BIT_MASK_TABLE1[] = {0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U};
const uint8_t BIT_MASK_TABLE2[] = {0xFEU, 0xFDU, 0xFBU, 0xF7U, 0xEFU, 0xDFU, 0xBFU, 0x7FU};
const uint8_t BIT_MASK_TABLE3[] = {0x01U, 0x02U, 0x04U, 0x08U, 0x10U, 0x20U, 0x40U, 0x80U};

#define WRITE_BIT1(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE1[(i)&7]) : (p[(i)>>3] & BIT_MASK_TABLE0[(i)&7])
#define READ_BIT1(p,i)    (p[(i)>>3] & BIT_MASK_TABLE1[(i)&7])

#define WRITE_BIT2(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE3[(i)&7]) : (p[(i)>>3] & BIT_MASK_TABLE2[(i)&7])
#define READ_BIT2(p,i)    (p[(i)>>3] & BIT_MASK_TABLE3[(i)&7])

const uint8_t INTERLEAVE_TABLE_RX[] = {
  0x00U, 0x00U, 0x03U, 0x00U, 0x06U, 0x00U, 0x09U, 0x00U, 0x0CU, 0x00U,
  0x0FU, 0x00U, 0x12U, 0x00U, 0x15U, 0x00U, 0x18U, 0x00U, 0x1BU, 0x00U,
  0x1EU, 0x00U, 0x21U, 0x00U, 0x24U, 0x00U, 0x27U, 0x00U, 0x2AU, 0x00U,
  0x2DU, 0x00U, 0x30U, 0x00U, 0x33U, 0x00U, 0x36U, 0x00U, 0x39U, 0x00U,
  0x3CU, 0x00U, 0x3FU, 0x00U, 0x42U, 0x00U, 0x45U, 0x00U, 0x48U, 0x00U,
  0x4BU, 0x00U, 0x4EU, 0x00U, 0x51U, 0x00U, 0x00U, 0x01U, 0x03U, 0x01U,
  0x06U, 0x01U, 0x09U, 0x01U, 0x0CU, 0x01U, 0x0FU, 0x01U, 0x12U, 0x01U,
  0x15U, 0x01U, 0x18U, 0x01U, 0x1BU, 0x01U, 0x1EU, 0x01U, 0x21U, 0x01U,
  0x24U, 0x01U, 0x27U, 0x01U, 0x2AU, 0x01U, 0x2DU, 0x01U, 0x30U, 0x01U,
  0x33U, 0x01U, 0x36U, 0x01U, 0x39U, 0x01U, 0x3CU, 0x01U, 0x3FU, 0x01U,
  0x42U, 0x01U, 0x45U, 0x01U, 0x48U, 0x01U, 0x4BU, 0x01U, 0x4EU, 0x01U,
  0x51U, 0x01U, 0x00U, 0x02U, 0x03U, 0x02U, 0x06U, 0x02U, 0x09U, 0x02U,
  0x0CU, 0x02U, 0x0FU, 0x02U, 0x12U, 0x02U, 0x15U, 0x02U, 0x18U, 0x02U,
  0x1BU, 0x02U, 0x1EU, 0x02U, 0x21U, 0x02U, 0x24U, 0x02U, 0x27U, 0x02U,
  0x2AU, 0x02U, 0x2DU, 0x02U, 0x30U, 0x02U, 0x33U, 0x02U, 0x36U, 0x02U,
  0x39U, 0x02U, 0x3CU, 0x02U, 0x3FU, 0x02U, 0x42U, 0x02U, 0x45U, 0x02U,
  0x48U, 0x02U, 0x4BU, 0x02U, 0x4EU, 0x02U, 0x51U, 0x02U, 0x00U, 0x03U,
  0x03U, 0x03U, 0x06U, 0x03U, 0x09U, 0x03U, 0x0CU, 0x03U, 0x0FU, 0x03U,
  0x12U, 0x03U, 0x15U, 0x03U, 0x18U, 0x03U, 0x1BU, 0x03U, 0x1EU, 0x03U,
  0x21U, 0x03U, 0x24U, 0x03U, 0x27U, 0x03U, 0x2AU, 0x03U, 0x2DU, 0x03U,
  0x30U, 0x03U, 0x33U, 0x03U, 0x36U, 0x03U, 0x39U, 0x03U, 0x3CU, 0x03U,
  0x3FU, 0x03U, 0x42U, 0x03U, 0x45U, 0x03U, 0x48U, 0x03U, 0x4BU, 0x03U,
  0x4EU, 0x03U, 0x51U, 0x03U, 0x00U, 0x04U, 0x03U, 0x04U, 0x06U, 0x04U,
  0x09U, 0x04U, 0x0CU, 0x04U, 0x0FU, 0x04U, 0x12U, 0x04U, 0x15U, 0x04U,
  0x18U, 0x04U, 0x1BU, 0x04U, 0x1EU, 0x04U, 0x21U, 0x04U, 0x24U, 0x04U,
  0x27U, 0x04U, 0x2AU, 0x04U, 0x2DU, 0x04U, 0x30U, 0x04U, 0x33U, 0x04U,
  0x36U, 0x04U, 0x39U, 0x04U, 0x3CU, 0x04U, 0x3FU, 0x04U, 0x42U, 0x04U,
  0x45U, 0x04U, 0x48U, 0x04U, 0x4BU, 0x04U, 0x4EU, 0x04U, 0x51U, 0x04U,
  0x00U, 0x05U, 0x03U, 0x05U, 0x06U, 0x05U, 0x09U, 0x05U, 0x0CU, 0x05U,
  0x0FU, 0x05U, 0x12U, 0x05U, 0x15U, 0x05U, 0x18U, 0x05U, 0x1BU, 0x05U,
  0x1EU, 0x05U, 0x21U, 0x05U, 0x24U, 0x05U, 0x27U, 0x05U, 0x2AU, 0x05U,
  0x2DU, 0x05U, 0x30U, 0x05U, 0x33U, 0x05U, 0x36U, 0x05U, 0x39U, 0x05U,
  0x3CU, 0x05U, 0x3FU, 0x05U, 0x42U, 0x05U, 0x45U, 0x05U, 0x48U, 0x05U,
  0x4BU, 0x05U, 0x4EU, 0x05U, 0x51U, 0x05U, 0x00U, 0x06U, 0x03U, 0x06U,
  0x06U, 0x06U, 0x09U, 0x06U, 0x0CU, 0x06U, 0x0FU, 0x06U, 0x12U, 0x06U,
  0x15U, 0x06U, 0x18U, 0x06U, 0x1BU, 0x06U, 0x1EU, 0x06U, 0x21U, 0x06U,
  0x24U, 0x06U, 0x27U, 0x06U, 0x2AU, 0x06U, 0x2DU, 0x06U, 0x30U, 0x06U,
  0x33U, 0x06U, 0x36U, 0x06U, 0x39U, 0x06U, 0x3CU, 0x06U, 0x3FU, 0x06U,
  0x42U, 0x06U, 0x45U, 0x06U, 0x48U, 0x06U, 0x4BU, 0x06U, 0x4EU, 0x06U,
  0x51U, 0x06U, 0x00U, 0x07U, 0x03U, 0x07U, 0x06U, 0x07U, 0x09U, 0x07U,
  0x0CU, 0x07U, 0x0FU, 0x07U, 0x12U, 0x07U, 0x15U, 0x07U, 0x18U, 0x07U,
  0x1BU, 0x07U, 0x1EU, 0x07U, 0x21U, 0x07U, 0x24U, 0x07U, 0x27U, 0x07U,
  0x2AU, 0x07U, 0x2DU, 0x07U, 0x30U, 0x07U, 0x33U, 0x07U, 0x36U, 0x07U,
  0x39U, 0x07U, 0x3CU, 0x07U, 0x3FU, 0x07U, 0x42U, 0x07U, 0x45U, 0x07U,
  0x48U, 0x07U, 0x4BU, 0x07U, 0x4EU, 0x07U, 0x51U, 0x07U, 0x01U, 0x00U,
  0x04U, 0x00U, 0x07U, 0x00U, 0x0AU, 0x00U, 0x0DU, 0x00U, 0x10U, 0x00U,
  0x13U, 0x00U, 0x16U, 0x00U, 0x19U, 0x00U, 0x1CU, 0x00U, 0x1FU, 0x00U,
  0x22U, 0x00U, 0x25U, 0x00U, 0x28U, 0x00U, 0x2BU, 0x00U, 0x2EU, 0x00U,
  0x31U, 0x00U, 0x34U, 0x00U, 0x37U, 0x00U, 0x3AU, 0x00U, 0x3DU, 0x00U,
  0x40U, 0x00U, 0x43U, 0x00U, 0x46U, 0x00U, 0x49U, 0x00U, 0x4CU, 0x00U,
  0x4FU, 0x00U, 0x52U, 0x00U, 0x01U, 0x01U, 0x04U, 0x01U, 0x07U, 0x01U,
  0x0AU, 0x01U, 0x0DU, 0x01U, 0x10U, 0x01U, 0x13U, 0x01U, 0x16U, 0x01U,
  0x19U, 0x01U, 0x1CU, 0x01U, 0x1FU, 0x01U, 0x22U, 0x01U, 0x25U, 0x01U,
  0x28U, 0x01U, 0x2BU, 0x01U, 0x2EU, 0x01U, 0x31U, 0x01U, 0x34U, 0x01U,
  0x37U, 0x01U, 0x3AU, 0x01U, 0x3DU, 0x01U, 0x40U, 0x01U, 0x43U, 0x01U,
  0x46U, 0x01U, 0x49U, 0x01U, 0x4CU, 0x01U, 0x4FU, 0x01U, 0x52U, 0x01U,
  0x01U, 0x02U, 0x04U, 0x02U, 0x07U, 0x02U, 0x0AU, 0x02U, 0x0DU, 0x02U,
  0x10U, 0x02U, 0x13U, 0x02U, 0x16U, 0x02U, 0x19U, 0x02U, 0x1CU, 0x02U,
  0x1FU, 0x02U, 0x22U, 0x02U, 0x25U, 0x02U, 0x28U, 0x02U, 0x2BU, 0x02U,
  0x2EU, 0x02U, 0x31U, 0x02U, 0x34U, 0x02U, 0x37U, 0x02U, 0x3AU, 0x02U,
  0x3DU, 0x02U, 0x40U, 0x02U, 0x43U, 0x02U, 0x46U, 0x02U, 0x49U, 0x02U,
  0x4CU, 0x02U, 0x4FU, 0x02U, 0x52U, 0x02U, 0x01U, 0x03U, 0x04U, 0x03U,
  0x07U, 0x03U, 0x0AU, 0x03U, 0x0DU, 0x03U, 0x10U, 0x03U, 0x13U, 0x03U,
  0x16U, 0x03U, 0x19U, 0x03U, 0x1CU, 0x03U, 0x1FU, 0x03U, 0x22U, 0x03U,
  0x25U, 0x03U, 0x28U, 0x03U, 0x2BU, 0x03U, 0x2EU, 0x03U, 0x31U, 0x03U,
  0x34U, 0x03U, 0x37U, 0x03U, 0x3AU, 0x03U, 0x3DU, 0x03U, 0x40U, 0x03U,
  0x43U, 0x03U, 0x46U, 0x03U, 0x49U, 0x03U, 0x4CU, 0x03U, 0x4FU, 0x03U,
  0x52U, 0x03U, 0x01U, 0x04U, 0x04U, 0x04U, 0x07U, 0x04U, 0x0AU, 0x04U,
  0x0DU, 0x04U, 0x10U, 0x04U, 0x13U, 0x04U, 0x16U, 0x04U, 0x19U, 0x04U,
  0x1CU, 0x04U, 0x1FU, 0x04U, 0x22U, 0x04U, 0x25U, 0x04U, 0x28U, 0x04U,
  0x2BU, 0x04U, 0x2EU, 0x04U, 0x31U, 0x04U, 0x34U, 0x04U, 0x37U, 0x04U,
  0x3AU, 0x04U, 0x3DU, 0x04U, 0x40U, 0x04U, 0x43U, 0x04U, 0x46U, 0x04U,
  0x49U, 0x04U, 0x4CU, 0x04U, 0x4FU, 0x04U, 0x01U, 0x05U, 0x04U, 0x05U,
  0x07U, 0x05U, 0x0AU, 0x05U, 0x0DU, 0x05U, 0x10U, 0x05U, 0x13U, 0x05U,
  0x16U, 0x05U, 0x19U, 0x05U, 0x1CU, 0x05U, 0x1FU, 0x05U, 0x22U, 0x05U,
  0x25U, 0x05U, 0x28U, 0x05U, 0x2BU, 0x05U, 0x2EU, 0x05U, 0x31U, 0x05U,
  0x34U, 0x05U, 0x37U, 0x05U, 0x3AU, 0x05U, 0x3DU, 0x05U, 0x40U, 0x05U,
  0x43U, 0x05U, 0x46U, 0x05U, 0x49U, 0x05U, 0x4CU, 0x05U, 0x4FU, 0x05U,
  0x01U, 0x06U, 0x04U, 0x06U, 0x07U, 0x06U, 0x0AU, 0x06U, 0x0DU, 0x06U,
  0x10U, 0x06U, 0x13U, 0x06U, 0x16U, 0x06U, 0x19U, 0x06U, 0x1CU, 0x06U,
  0x1FU, 0x06U, 0x22U, 0x06U, 0x25U, 0x06U, 0x28U, 0x06U, 0x2BU, 0x06U,
  0x2EU, 0x06U, 0x31U, 0x06U, 0x34U, 0x06U, 0x37U, 0x06U, 0x3AU, 0x06U,
  0x3DU, 0x06U, 0x40U, 0x06U, 0x43U, 0x06U, 0x46U, 0x06U, 0x49U, 0x06U,
  0x4CU, 0x06U, 0x4FU, 0x06U, 0x01U, 0x07U, 0x04U, 0x07U, 0x07U, 0x07U,
  0x0AU, 0x07U, 0x0DU, 0x07U, 0x10U, 0x07U, 0x13U, 0x07U, 0x16U, 0x07U,
  0x19U, 0x07U, 0x1CU, 0x07U, 0x1FU, 0x07U, 0x22U, 0x07U, 0x25U, 0x07U,
  0x28U, 0x07U, 0x2BU, 0x07U, 0x2EU, 0x07U, 0x31U, 0x07U, 0x34U, 0x07U,
  0x37U, 0x07U, 0x3AU, 0x07U, 0x3DU, 0x07U, 0x40U, 0x07U, 0x43U, 0x07U,
  0x46U, 0x07U, 0x49U, 0x07U, 0x4CU, 0x07U, 0x4FU, 0x07U, 0x02U, 0x00U,
  0x05U, 0x00U, 0x08U, 0x00U, 0x0BU, 0x00U, 0x0EU, 0x00U, 0x11U, 0x00U,
  0x14U, 0x00U, 0x17U, 0x00U, 0x1AU, 0x00U, 0x1DU, 0x00U, 0x20U, 0x00U,
  0x23U, 0x00U, 0x26U, 0x00U, 0x29U, 0x00U, 0x2CU, 0x00U, 0x2FU, 0x00U,
  0x32U, 0x00U, 0x35U, 0x00U, 0x38U, 0x00U, 0x3BU, 0x00U, 0x3EU, 0x00U,
  0x41U, 0x00U, 0x44U, 0x00U, 0x47U, 0x00U, 0x4AU, 0x00U, 0x4DU, 0x00U,
  0x50U, 0x00U, 0x02U, 0x01U, 0x05U, 0x01U, 0x08U, 0x01U, 0x0BU, 0x01U,
  0x0EU, 0x01U, 0x11U, 0x01U, 0x14U, 0x01U, 0x17U, 0x01U, 0x1AU, 0x01U,
  0x1DU, 0x01U, 0x20U, 0x01U, 0x23U, 0x01U, 0x26U, 0x01U, 0x29U, 0x01U,
  0x2CU, 0x01U, 0x2FU, 0x01U, 0x32U, 0x01U, 0x35U, 0x01U, 0x38U, 0x01U,
  0x3BU, 0x01U, 0x3EU, 0x01U, 0x41U, 0x01U, 0x44U, 0x01U, 0x47U, 0x01U,
  0x4AU, 0x01U, 0x4DU, 0x01U, 0x50U, 0x01U, 0x02U, 0x02U, 0x05U, 0x02U,
  0x08U, 0x02U, 0x0BU, 0x02U, 0x0EU, 0x02U, 0x11U, 0x02U, 0x14U, 0x02U,
  0x17U, 0x02U, 0x1AU, 0x02U, 0x1DU, 0x02U, 0x20U, 0x02U, 0x23U, 0x02U,
  0x26U, 0x02U, 0x29U, 0x02U, 0x2CU, 0x02U, 0x2FU, 0x02U, 0x32U, 0x02U,
  0x35U, 0x02U, 0x38U, 0x02U, 0x3BU, 0x02U, 0x3EU, 0x02U, 0x41U, 0x02U,
  0x44U, 0x02U, 0x47U, 0x02U, 0x4AU, 0x02U, 0x4DU, 0x02U, 0x50U, 0x02U,
  0x02U, 0x03U, 0x05U, 0x03U, 0x08U, 0x03U, 0x0BU, 0x03U, 0x0EU, 0x03U,
  0x11U, 0x03U, 0x14U, 0x03U, 0x17U, 0x03U, 0x1AU, 0x03U, 0x1DU, 0x03U,
  0x20U, 0x03U, 0x23U, 0x03U, 0x26U, 0x03U, 0x29U, 0x03U, 0x2CU, 0x03U,
  0x2FU, 0x03U, 0x32U, 0x03U, 0x35U, 0x03U, 0x38U, 0x03U, 0x3BU, 0x03U,
  0x3EU, 0x03U, 0x41U, 0x03U, 0x44U, 0x03U, 0x47U, 0x03U, 0x4AU, 0x03U,
  0x4DU, 0x03U, 0x50U, 0x03U, 0x02U, 0x04U, 0x05U, 0x04U, 0x08U, 0x04U,
  0x0BU, 0x04U, 0x0EU, 0x04U, 0x11U, 0x04U, 0x14U, 0x04U, 0x17U, 0x04U,
  0x1AU, 0x04U, 0x1DU, 0x04U, 0x20U, 0x04U, 0x23U, 0x04U, 0x26U, 0x04U,
  0x29U, 0x04U, 0x2CU, 0x04U, 0x2FU, 0x04U, 0x32U, 0x04U, 0x35U, 0x04U,
  0x38U, 0x04U, 0x3BU, 0x04U, 0x3EU, 0x04U, 0x41U, 0x04U, 0x44U, 0x04U,
  0x47U, 0x04U, 0x4AU, 0x04U, 0x4DU, 0x04U, 0x50U, 0x04U, 0x02U, 0x05U,
  0x05U, 0x05U, 0x08U, 0x05U, 0x0BU, 0x05U, 0x0EU, 0x05U, 0x11U, 0x05U,
  0x14U, 0x05U, 0x17U, 0x05U, 0x1AU, 0x05U, 0x1DU, 0x05U, 0x20U, 0x05U,
  0x23U, 0x05U, 0x26U, 0x05U, 0x29U, 0x05U, 0x2CU, 0x05U, 0x2FU, 0x05U,
  0x32U, 0x05U, 0x35U, 0x05U, 0x38U, 0x05U, 0x3BU, 0x05U, 0x3EU, 0x05U,
  0x41U, 0x05U, 0x44U, 0x05U, 0x47U, 0x05U, 0x4AU, 0x05U, 0x4DU, 0x05U,
  0x50U, 0x05U, 0x02U, 0x06U, 0x05U, 0x06U, 0x08U, 0x06U, 0x0BU, 0x06U,
  0x0EU, 0x06U, 0x11U, 0x06U, 0x14U, 0x06U, 0x17U, 0x06U, 0x1AU, 0x06U,
  0x1DU, 0x06U, 0x20U, 0x06U, 0x23U, 0x06U, 0x26U, 0x06U, 0x29U, 0x06U,
  0x2CU, 0x06U, 0x2FU, 0x06U, 0x32U, 0x06U, 0x35U, 0x06U, 0x38U, 0x06U,
  0x3BU, 0x06U, 0x3EU, 0x06U, 0x41U, 0x06U, 0x44U, 0x06U, 0x47U, 0x06U,
  0x4AU, 0x06U, 0x4DU, 0x06U, 0x50U, 0x06U, 0x02U, 0x07U, 0x05U, 0x07U,
  0x08U, 0x07U, 0x0BU, 0x07U, 0x0EU, 0x07U, 0x11U, 0x07U, 0x14U, 0x07U,
  0x17U, 0x07U, 0x1AU, 0x07U, 0x1DU, 0x07U, 0x20U, 0x07U, 0x23U, 0x07U,
  0x26U, 0x07U, 0x29U, 0x07U, 0x2CU, 0x07U, 0x2FU, 0x07U, 0x32U, 0x07U,
  0x35U, 0x07U, 0x38U, 0x07U, 0x3BU, 0x07U, 0x3EU, 0x07U, 0x41U, 0x07U,
  0x44U, 0x07U, 0x47U, 0x07U, 0x4AU, 0x07U, 0x4DU, 0x07U, 0x50U, 0x07U,
};

const uint8_t SCRAMBLE_TABLE_RX[] = {
  0x70U, 0x4FU, 0x93U, 0x40U, 0x64U, 0x74U, 0x6DU, 0x30U, 0x2BU, 0xE7U,
  0x2DU, 0x54U, 0x5FU, 0x8AU, 0x1DU, 0x7FU, 0xB8U, 0xA7U, 0x49U, 0x20U,
  0x32U, 0xBAU, 0x36U, 0x98U, 0x95U, 0xF3U, 0x16U, 0xAAU, 0x2FU, 0xC5U,
  0x8EU, 0x3FU, 0xDCU, 0xD3U, 0x24U, 0x10U, 0x19U, 0x5DU, 0x1BU, 0xCCU,
  0xCAU, 0x79U, 0x0BU, 0xD5U, 0x97U, 0x62U, 0xC7U, 0x1FU, 0xEEU, 0x69U,
  0x12U, 0x88U, 0x8CU, 0xAEU, 0x0DU, 0x66U, 0xE5U, 0xBCU, 0x85U, 0xEAU,
  0x4BU, 0xB1U, 0xE3U, 0x0FU, 0xF7U, 0x34U, 0x09U, 0x44U, 0x46U, 0xD7U,
  0x06U, 0xB3U, 0x72U, 0xDEU, 0x42U, 0xF5U, 0xA5U, 0xD8U, 0xF1U, 0x87U,
  0x7BU, 0x9AU, 0x04U, 0x22U, 0xA3U, 0x6BU, 0x83U, 0x59U, 0x39U, 0x6FU,
  0x00U};

const uint16_t CCITT_TABLE[] = {
  0x0000U, 0x1189U, 0x2312U, 0x329bU, 0x4624U, 0x57adU, 0x6536U, 0x74bfU,
  0x8c48U, 0x9dc1U, 0xaf5aU, 0xbed3U, 0xca6cU, 0xdbe5U, 0xe97eU, 0xf8f7U,
  0x1081U, 0x0108U, 0x3393U, 0x221aU, 0x56a5U, 0x472cU, 0x75b7U, 0x643eU,
  0x9cc9U, 0x8d40U, 0xbfdbU, 0xae52U, 0xdaedU, 0xcb64U, 0xf9ffU, 0xe876U,
  0x2102U, 0x308bU, 0x0210U, 0x1399U, 0x6726U, 0x76afU, 0x4434U, 0x55bdU,
  0xad4aU, 0xbcc3U, 0x8e58U, 0x9fd1U, 0xeb6eU, 0xfae7U, 0xc87cU, 0xd9f5U,
  0x3183U, 0x200aU, 0x1291U, 0x0318U, 0x77a7U, 0x662eU, 0x54b5U, 0x453cU,
  0xbdcbU, 0xac42U, 0x9ed9U, 0x8f50U, 0xfbefU, 0xea66U, 0xd8fdU, 0xc974U,
  0x4204U, 0x538dU, 0x6116U, 0x709fU, 0x0420U, 0x15a9U, 0x2732U, 0x36bbU,
  0xce4cU, 0xdfc5U, 0xed5eU, 0xfcd7U, 0x8868U, 0x99e1U, 0xab7aU, 0xbaf3U,
  0x5285U, 0x430cU, 0x7197U, 0x601eU, 0x14a1U, 0x0528U, 0x37b3U, 0x263aU,
  0xdecdU, 0xcf44U, 0xfddfU, 0xec56U, 0x98e9U, 0x8960U, 0xbbfbU, 0xaa72U,
  0x6306U, 0x728fU, 0x4014U, 0x519dU, 0x2522U, 0x34abU, 0x0630U, 0x17b9U,
  0xef4eU, 0xfec7U, 0xcc5cU, 0xddd5U, 0xa96aU, 0xb8e3U, 0x8a78U, 0x9bf1U,
  0x7387U, 0x620eU, 0x5095U, 0x411cU, 0x35a3U, 0x242aU, 0x16b1U, 0x0738U,
  0xffcfU, 0xee46U, 0xdcddU, 0xcd54U, 0xb9ebU, 0xa862U, 0x9af9U, 0x8b70U,
  0x8408U, 0x9581U, 0xa71aU, 0xb693U, 0xc22cU, 0xd3a5U, 0xe13eU, 0xf0b7U,
  0x0840U, 0x19c9U, 0x2b52U, 0x3adbU, 0x4e64U, 0x5fedU, 0x6d76U, 0x7cffU,
  0x9489U, 0x8500U, 0xb79bU, 0xa612U, 0xd2adU, 0xc324U, 0xf1bfU, 0xe036U,
  0x18c1U, 0x0948U, 0x3bd3U, 0x2a5aU, 0x5ee5U, 0x4f6cU, 0x7df7U, 0x6c7eU,
  0xa50aU, 0xb483U, 0x8618U, 0x9791U, 0xe32eU, 0xf2a7U, 0xc03cU, 0xd1b5U,
  0x2942U, 0x38cbU, 0x0a50U, 0x1bd9U, 0x6f66U, 0x7eefU, 0x4c74U, 0x5dfdU,
  0xb58bU, 0xa402U, 0x9699U, 0x8710U, 0xf3afU, 0xe226U, 0xd0bdU, 0xc134U,
  0x39c3U, 0x284aU, 0x1ad1U, 0x0b58U, 0x7fe7U, 0x6e6eU, 0x5cf5U, 0x4d7cU,
  0xc60cU, 0xd785U, 0xe51eU, 0xf497U, 0x8028U, 0x91a1U, 0xa33aU, 0xb2b3U,
  0x4a44U, 0x5bcdU, 0x6956U, 0x78dfU, 0x0c60U, 0x1de9U, 0x2f72U, 0x3efbU,
  0xd68dU, 0xc704U, 0xf59fU, 0xe416U, 0x90a9U, 0x8120U, 0xb3bbU, 0xa232U,
  0x5ac5U, 0x4b4cU, 0x79d7U, 0x685eU, 0x1ce1U, 0x0d68U, 0x3ff3U, 0x2e7aU,
  0xe70eU, 0xf687U, 0xc41cU, 0xd595U, 0xa12aU, 0xb0a3U, 0x8238U, 0x93b1U,
  0x6b46U, 0x7acfU, 0x4854U, 0x59ddU, 0x2d62U, 0x3cebU, 0x0e70U, 0x1ff9U,
  0xf78fU, 0xe606U, 0xd49dU, 0xc514U, 0xb1abU, 0xa022U, 0x92b9U, 0x8330U,
  0x7bc7U, 0x6a4eU, 0x58d5U, 0x495cU, 0x3de3U, 0x2c6aU, 0x1ef1U, 0x0f78U};

const uint16_t NOENDPTR = 9999U;

// Fraction (in 1/256) of the amplitude of a symbol that spreads into each of its two neighbours
// through the Gaussian filters of the transmitter and of the receiver (about 0.105)
const int32_t ISI_FACTOR = 27;

// If the header checksum fails but the Viterbi path metric says the signal looks like a header
// (random noise gives about 45000, a clean header 33000 to 37000), the header is decoded again
// with the sub-sample position moved by these amounts (in 1/256 samples), as the timing
// estimate from the sync can be slightly off in noise
const int HEADER_RETRY_METRIC = 40000;
const int16_t HEADER_RETRY_ADJ[] = {-96, +96, -192, +192};

// Soft metric: 0 = certain 0 bit, SOFT_MAX = certain 1 bit
const int SOFT_MAX = 254;

// Length of the header sample buffer
const uint16_t HEADER_BUFFER_SAMPLES = DSTAR_FEC_SECTION_LENGTH_SAMPLES + 2U * DSTAR_RADIO_SYMBOL_LENGTH;

CDStarRX::CDStarRX() :
m_rxState(DSRXS_NONE),
m_bitBuffer(),
m_headerBuffer(),
m_dataBuffer(),
m_bitPtr(0U),
m_headerPtr(0U),
m_dataPtr(0U),
m_startPtr(NOENDPTR),
m_syncPtr(NOENDPTR),
m_minSyncPtr(NOENDPTR),
m_maxSyncPtr(NOENDPTR),
m_maxFrameCorr(0),
m_maxDataCorr(0),
m_frameCount(0U),
m_countdown(0U),
m_mar(0U),
m_pathMetric(),
m_pathMemory0(),
m_pathMemory1(),
m_pathMemory2(),
m_pathMemory3(),
m_fecOutput(),
m_rssiAccum(0U),
m_rssiCount(0U),
m_framePtr(0U),
m_deltaPending(false),
m_dataSynced(false),
m_delta(0),
m_headerMetric(0)
{
}

void CDStarRX::reset()
{
  m_rxState      = DSRXS_NONE;
  m_headerPtr    = 0U;
  m_dataPtr      = 0U;
  m_bitPtr       = 0U;
  m_maxFrameCorr = 0;
  m_maxDataCorr  = 0;
  m_startPtr     = NOENDPTR;
  m_syncPtr      = NOENDPTR;
  m_minSyncPtr   = NOENDPTR;
  m_maxSyncPtr   = NOENDPTR;
  m_frameCount   = 0U;
  m_countdown    = 0U;
  m_rssiAccum    = 0U;
  m_rssiCount    = 0U;
  m_deltaPending = false;
  m_dataSynced   = false;
  m_delta        = 0;
}

void CDStarRX::samples(const q15_t* samples, const uint16_t* rssi, uint8_t length)
{
  for (uint16_t i = 0U; i < length; i++) {
    m_rssiAccum += rssi[i];
    m_rssiCount++;

    q15_t sample = samples[i];

    m_bitBuffer[m_bitPtr] <<= 1;
    if (sample < 0)
      m_bitBuffer[m_bitPtr] |= 0x01U;

    m_dataBuffer[m_dataPtr] = sample;

    switch (m_rxState) {
      case DSRXS_HEADER:
        processHeader(sample);
        break;
      case DSRXS_DATA:
        processData();
        break;
      default:
        processNone(sample);
        break;
    }

    m_dataPtr++;
    if (m_dataPtr >= DSTAR_DATA_LENGTH_SAMPLES)
      m_dataPtr = 0U;

    m_bitPtr++;
    if (m_bitPtr >= DSTAR_RADIO_SYMBOL_LENGTH)
      m_bitPtr = 0U;
  }
}

void CDStarRX::processNone(q15_t sample)
{
  // Fuzzy matching of the frame sync sequence
  bool ret = correlateFrameSync();
  if (ret) {
    m_countdown    = 5U;
    m_deltaPending = true;
    m_delta        = 0;

    m_headerBuffer[m_headerPtr] = sample;
    m_headerPtr++;

    m_rssiAccum = 0U;
    m_rssiCount = 0U;

    m_rxState = DSRXS_HEADER;

    return;
  }

  // Fuzzy matching of the data sync bit sequence
  ret = correlateDataSync();
  if (ret) {
    DEBUG1("DStarRX: found data sync in None");

    io.setDecode(true);
    io.setADCDetection(true);

    m_rxState = DSRXS_DATA;
  }
}

void CDStarRX::processHeader(q15_t sample)
{
  // Keep looking for a better frame sync while the header is being collected. The first match
  // can be a false one inside the alternating preamble, which looks like the start of the sync,
  // and it would otherwise hide the real sync that follows. A better match restarts the header.
  correlateFrameSync();

  if (m_countdown > 0U) {
    m_countdown--;
  } else if (m_deltaPending) {
    // The best sync position is now known and the sample after it is available:
    // find the sub-sample position of the sync, used to interpolate the header and data symbols
    m_delta        = syncDelta(m_framePtr, true);
    m_deltaPending = false;
  }

  m_headerBuffer[m_headerPtr] = sample;
  m_headerPtr++;

  // A full FEC header
  if (m_headerPtr == (DSTAR_FEC_SECTION_LENGTH_SAMPLES + DSTAR_RADIO_SYMBOL_LENGTH)) {
    // Descrambling, deinterleaving and soft Viterbi work directly on the samples,
    // then return true if the checksum was correct
    uint8_t header[DSTAR_HEADER_LENGTH_BYTES];
    bool ok = rxHeader(m_headerBuffer, header);
    if (!ok && m_headerMetric < HEADER_RETRY_METRIC) {
      for (uint8_t a = 0U; !ok && a < 4U; a++)
        ok = rxHeader(m_headerBuffer, header, HEADER_RETRY_ADJ[a]);
    }
    if (!ok) {
      // The checksum failed, return to looking for syncs
      m_rxState = DSRXS_NONE;
      m_maxFrameCorr = 0;
      m_maxDataCorr  = 0;
    } else {
      io.setDecode(true);
      io.setADCDetection(true);

      writeRSSIHeader(header);
    }
  }

  // Ready to start the first data section
  if (m_headerPtr == (DSTAR_FEC_SECTION_LENGTH_SAMPLES + 2U * DSTAR_RADIO_SYMBOL_LENGTH)) {
    m_frameCount = 0U;
    m_dataPtr    = 0U;

    m_startPtr   = 476U;
    m_syncPtr    = 471U;
    m_maxSyncPtr = 472U;
    m_minSyncPtr = 470U;

    DEBUG5("DStarRX: calc start/sync/max/min", m_startPtr, m_syncPtr, m_maxSyncPtr, m_minSyncPtr);

    m_rxState = DSRXS_DATA;
  }
}

void CDStarRX::processData()
{
  // Fuzzy matching of the end frame sequences
  if (countBits64((m_bitBuffer[m_bitPtr] & DSTAR_END_SYNC_MASK) ^ DSTAR_END_SYNC_DATA) <= END_SYNC_ERRS) {
    DEBUG1("DStarRX: Found end sync in Data");

    io.setDecode(false);
    io.setADCDetection(false);

    serial.writeDStarEOT();

    m_maxFrameCorr = 0;
    m_maxDataCorr  = 0;

    m_rxState = DSRXS_NONE;
    return;
  }

  // Fuzzy matching of the data sync bit sequence
  if (m_minSyncPtr < m_maxSyncPtr) {
    if (m_dataPtr >= m_minSyncPtr && m_dataPtr <= m_maxSyncPtr)
      correlateDataSync();
  } else {
    if (m_dataPtr >= m_minSyncPtr || m_dataPtr <= m_maxSyncPtr)
      correlateDataSync();
  }

  // We've not seen a data sync for too long, signal RXLOST and change to RX_NONE
  if (m_frameCount >= MAX_FRAMES) {
    DEBUG1("DStarRX: data sync timed out, lost lock");

    io.setDecode(false);
    io.setADCDetection(false);

    serial.writeDStarLost();

    m_maxFrameCorr = 0;
    m_maxDataCorr  = 0;

    m_rxState = DSRXS_NONE;
    return;
  }

  // Send a data frame to the host if the required number of bits have been received
  if (m_dataPtr == m_maxSyncPtr) {
    // A data sync was found for this frame, so update the sub-sample position
    if (m_dataSynced) {
      m_delta      = syncDelta(m_syncPtr, false);
      m_dataSynced = false;
    }

    uint8_t buffer[DSTAR_DATA_LENGTH_BYTES + 2U];
    samplesToBits(m_dataBuffer, m_startPtr, DSTAR_DATA_LENGTH_SYMBOLS, buffer, DSTAR_DATA_LENGTH_SAMPLES);

    if ((m_frameCount % 21U) == 0U) {
      if (m_frameCount == 0U) {
        buffer[9U]  = DSTAR_DATA_SYNC_BYTES[9U];
        buffer[10U] = DSTAR_DATA_SYNC_BYTES[10U];
        buffer[11U] = DSTAR_DATA_SYNC_BYTES[11U];
        DEBUG5("DStarRX: found start/sync/max/min", m_startPtr, m_syncPtr, m_maxSyncPtr, m_minSyncPtr);
      }

      writeRSSIData(buffer);
    } else {
      serial.writeDStarData(buffer, DSTAR_DATA_LENGTH_BYTES);
    }

    m_frameCount++;

    m_maxFrameCorr = 0;
    m_maxDataCorr  = 0;
  }
}

void CDStarRX::writeRSSIHeader(unsigned char* header)
{
#if defined(SEND_RSSI_DATA)
  if (m_rssiCount > 0U) {
    uint16_t rssi = m_rssiAccum / m_rssiCount;

    header[41U] = (rssi >> 8) & 0xFFU;
    header[42U] = (rssi >> 0) & 0xFFU;

    serial.writeDStarHeader(header, DSTAR_HEADER_LENGTH_BYTES + 2U);
  } else {
    serial.writeDStarHeader(header, DSTAR_HEADER_LENGTH_BYTES + 0U);
  }
#else
  serial.writeDStarHeader(header, DSTAR_HEADER_LENGTH_BYTES + 0U);
#endif

  m_rssiAccum = 0U;
  m_rssiCount = 0U;
}

void CDStarRX::writeRSSIData(unsigned char* data)
{
#if defined(SEND_RSSI_DATA)
  if (m_rssiCount > 0U) {
    uint16_t rssi = m_rssiAccum / m_rssiCount;

    data[12U] = (rssi >> 8) & 0xFFU;
    data[13U] = (rssi >> 0) & 0xFFU;

    serial.writeDStarData(data, DSTAR_DATA_LENGTH_BYTES + 2U);
  } else {
    serial.writeDStarData(data, DSTAR_DATA_LENGTH_BYTES + 0U);
  }
#else
  serial.writeDStarData(data, DSTAR_DATA_LENGTH_BYTES + 0U);
#endif

  m_rssiAccum = 0U;
  m_rssiCount = 0U;
}

bool CDStarRX::correlateFrameSync()
{
  if (countBits64((m_bitBuffer[m_bitPtr] & DSTAR_FRAME_SYNC_MASK) ^ DSTAR_FRAME_SYNC_DATA) <= FRAME_SYNC_ERRS) {
    uint16_t ptr = m_dataPtr + DSTAR_DATA_LENGTH_SAMPLES - DSTAR_FRAME_SYNC_LENGTH_SAMPLES + DSTAR_RADIO_SYMBOL_LENGTH;
    if (ptr >= DSTAR_DATA_LENGTH_SAMPLES)
      ptr -= DSTAR_DATA_LENGTH_SAMPLES;

    q31_t corr = 0;

    for (uint8_t i = 0U; i < DSTAR_FRAME_SYNC_LENGTH_SYMBOLS; i++) {
      q15_t val = m_dataBuffer[ptr];

      if (DSTAR_FRAME_SYNC_SYMBOLS[i])
        corr -= val;
      else
        corr += val;

      ptr += DSTAR_RADIO_SYMBOL_LENGTH;
      if (ptr >= DSTAR_DATA_LENGTH_SAMPLES)
        ptr -= DSTAR_DATA_LENGTH_SAMPLES;
    }

    if (corr > m_maxFrameCorr) {
      m_maxFrameCorr = corr;
      m_framePtr     = m_dataPtr;
      m_headerPtr    = 0U;
      m_countdown    = 5U;
      m_deltaPending = true;
      m_delta        = 0;
      return true;
    }
  }

  return false;
}

bool CDStarRX::correlateDataSync()
{
  uint8_t maxErrs = 0U;
  if (m_rxState == DSRXS_DATA)
    maxErrs = DATA_SYNC_ERRS;

  if (countBits64((m_bitBuffer[m_bitPtr] & DSTAR_DATA_SYNC_MASK) ^ DSTAR_DATA_SYNC_DATA) <= maxErrs) {
    uint16_t ptr = m_dataPtr + DSTAR_DATA_LENGTH_SAMPLES - DSTAR_DATA_SYNC_LENGTH_SAMPLES + DSTAR_RADIO_SYMBOL_LENGTH;
    if (ptr >= DSTAR_DATA_LENGTH_SAMPLES)
      ptr -= DSTAR_DATA_LENGTH_SAMPLES;

    q31_t corr = 0;

    for (uint8_t i = 0U; i < DSTAR_DATA_SYNC_LENGTH_SYMBOLS; i++) {
      q15_t val = m_dataBuffer[ptr];

      if (DSTAR_DATA_SYNC_SYMBOLS[i])
        corr -= val;
      else
        corr += val;

      ptr += DSTAR_RADIO_SYMBOL_LENGTH;
      if (ptr >= DSTAR_DATA_LENGTH_SAMPLES)
        ptr -= DSTAR_DATA_LENGTH_SAMPLES;
    }

    if (corr > m_maxDataCorr) {
      m_maxDataCorr = corr;
      m_frameCount  = 0U;
      m_dataSynced  = true;

      m_syncPtr    = m_dataPtr;

      m_startPtr   = m_dataPtr + DSTAR_RADIO_SYMBOL_LENGTH;
      if (m_startPtr >= DSTAR_DATA_LENGTH_SAMPLES)
        m_startPtr -= DSTAR_DATA_LENGTH_SAMPLES;

      m_maxSyncPtr = m_syncPtr + 1U;
      if (m_maxSyncPtr >= DSTAR_DATA_LENGTH_SAMPLES)
        m_maxSyncPtr -= DSTAR_DATA_LENGTH_SAMPLES;

      m_minSyncPtr = m_syncPtr + DSTAR_DATA_LENGTH_SAMPLES - 1U;
      if (m_minSyncPtr >= DSTAR_DATA_LENGTH_SAMPLES)
        m_minSyncPtr -= DSTAR_DATA_LENGTH_SAMPLES;

      return true;
    }
  }

  return false;
}

void CDStarRX::samplesToBits(const q15_t* inBuffer, uint16_t start, uint16_t count, uint8_t* outBuffer, uint16_t limit)
{
  // Mean amplitude of the symbols, used to scale the inter-symbol interference correction
  uint16_t ptr = start;
  q31_t sum = 0;
  for (uint16_t i = 0U; i < count; i++) {
    q31_t s = sampleAt(inBuffer, ptr, m_delta, limit);
    sum += (s < 0) ? -s : s;

    ptr += DSTAR_RADIO_SYMBOL_LENGTH;
    if (ptr >= limit)
      ptr -= limit;
  }
  q31_t isi = ((sum / count) * ISI_FACTOR) >> 8;

  // Each symbol picks up a fraction of its two neighbours (the Gaussian filters spread
  // each symbol over its neighbours). The hard decision of the neighbours is used to remove it.
  q31_t prev = 0, cur = 0, next = 0;
  ptr = start;
  cur = sampleAt(inBuffer, ptr, m_delta, limit);
  for (uint16_t i = 0U; i < count; i++) {
    uint16_t nptr = ptr + DSTAR_RADIO_SYMBOL_LENGTH;
    if (nptr >= limit)
      nptr -= limit;
    next = (i + 1U < count) ? sampleAt(inBuffer, nptr, m_delta, limit) : cur;

    q31_t v = cur;
    if (i > 0U && i + 1U < count) {
      q31_t n = ((prev < 0) ? -1 : 1) + ((next < 0) ? -1 : 1);
      v -= n * isi;
    }

    if (v < 0)
      WRITE_BIT2(outBuffer, i, true);
    else
      WRITE_BIT2(outBuffer, i, false);

    prev = cur;
    cur  = next;
    ptr  = nptr;
  }
}

// Value of the waveform at buffer position pos + delta/256 samples (|delta| <= 128),
// by quadratic (3 point Lagrange) interpolation. delta == 0 returns the stored sample.
q15_t CDStarRX::sampleAt(const q15_t* inBuffer, uint16_t pos, int16_t delta, uint16_t limit) const
{
  if (delta == 0)
    return inBuffer[pos];

  uint16_t prev = (pos == 0U) ? limit - 1U : pos - 1U;
  uint16_t next = (pos + 1U >= limit) ? 0U : pos + 1U;

  q31_t s0 = inBuffer[pos];
  q31_t sm = inBuffer[prev];
  q31_t sp = inBuffer[next];

  q31_t d = delta;
  q31_t v = s0 + ((d * (sp - sm)) >> 9) + (((d * d) * (sp - 2 * s0 + sm)) >> 17);

  if (v > 32767)
    v = 32767;
  if (v < -32768)
    v = -32768;

  return q15_t(v);
}

// Sub-sample position (1/256 sample units, -128..+128) of the correlation peak of the frame
// or data sync, relative to the sample 'end' (its last symbol), from a parabola through
// the correlation at -1, 0 and +1 samples.
int16_t CDStarRX::syncDelta(uint16_t end, bool frame) const
{
  const bool*    symbols = frame ? DSTAR_FRAME_SYNC_SYMBOLS : DSTAR_DATA_SYNC_SYMBOLS;
  const uint16_t length  = frame ? DSTAR_FRAME_SYNC_LENGTH_SYMBOLS : DSTAR_DATA_SYNC_LENGTH_SYMBOLS;
  const uint16_t samples = length * DSTAR_RADIO_SYMBOL_LENGTH;

  q31_t c[3U];
  for (uint8_t d = 0U; d < 3U; d++) {
    uint16_t ptr = (end + 2U * DSTAR_DATA_LENGTH_SAMPLES - samples + DSTAR_RADIO_SYMBOL_LENGTH + d - 1U) % DSTAR_DATA_LENGTH_SAMPLES;

    q31_t corr = 0;
    for (uint8_t i = 0U; i < length; i++) {
      q15_t val = m_dataBuffer[ptr];

      if (symbols[i])
        corr -= val;
      else
        corr += val;

      ptr += DSTAR_RADIO_SYMBOL_LENGTH;
      if (ptr >= DSTAR_DATA_LENGTH_SAMPLES)
        ptr -= DSTAR_DATA_LENGTH_SAMPLES;
    }

    c[d] = corr;
  }

  // Not a clear peak, don't interpolate
  q31_t den = c[0U] - 2 * c[1U] + c[2U];
  if (c[1U] < c[0U] || c[1U] < c[2U] || den >= 0)
    return 0;

  q31_t p = ((c[0U] - c[2U]) << 7) / den;
  if (p > 128)
    p = 128;
  if (p < -128)
    p = -128;

  return int16_t(p);
}

// Header sample at position pos, with the sub-sample position moved by adj/256 samples
q15_t CDStarRX::headerSample(const q15_t* in, int32_t pos, int16_t adj) const
{
  int32_t total = int32_t(m_delta) + adj;
  int32_t shift = 0;
  if (total > 128) {
    shift = 1;
    total -= 256;
  } else if (total < -128) {
    shift = -1;
    total += 256;
  }

  pos += shift;
  if (pos < 1)
    pos = 1;
  if (pos > int32_t(HEADER_BUFFER_SAMPLES) - 2)
    pos = HEADER_BUFFER_SAMPLES - 2;

  return sampleAt(in, uint16_t(pos), int16_t(total), HEADER_BUFFER_SAMPLES);
}

bool CDStarRX::rxHeader(const q15_t* in, uint8_t* out, int16_t adj)
{
  // The first header symbol is at sample DSTAR_RADIO_SYMBOL_LENGTH
  const uint16_t FIRST = DSTAR_RADIO_SYMBOL_LENGTH;

  // Mean symbol amplitude, used to normalise the soft values
  uint32_t sum = 0U;
  for (uint16_t i = 0U; i < 660U; i++) {
    int32_t s = headerSample(in, FIRST + i * DSTAR_RADIO_SYMBOL_LENGTH, adj);
    sum += (s < 0) ? -s : s;
  }

  int32_t amp = int32_t(sum / 660U);
  if (amp < 1)
    amp = 1;

  // Samples -> soft values, with descrambling and deinterleaving in a single pass.
  // soft[] is already in the order the convolutional decoder expects.
  uint8_t soft[660U];
  for (uint16_t i = 0U; i < 660U; i++)
    soft[i] = SOFT_MAX / 2;

  for (uint16_t i = 0U; i < 660U; i++) {
    // 85/amp: a symbol as large as the mean maps to 85, saturating at 1.5 times the mean.
    // This scaling is a starting point: try others (e.g. 127/amp).
    uint16_t pos = FIRST + i * DSTAR_RADIO_SYMBOL_LENGTH;
    int32_t cur  = headerSample(in, pos, adj);
    int32_t prv  = headerSample(in, pos - DSTAR_RADIO_SYMBOL_LENGTH, adj);
    int32_t nxt  = headerSample(in, pos + DSTAR_RADIO_SYMBOL_LENGTH, adj);

    // Remove the part of the neighbouring symbols that spreads into this one
    cur -= (((prv < 0) ? -1 : 1) + ((nxt < 0) ? -1 : 1)) * ((amp * ISI_FACTOR) >> 8);

    int32_t x = (cur * 85) / amp;
    if (x > 127)
      x = 127;
    if (x < -127)
      x = -127;

    // sample < 0 => bit 1 => high value
    int s = 127 - x;

    // Descrambling: a scramble bit of 1 inverts the bit
    if (SCRAMBLE_TABLE_RX[i >> 3] & (0x01U << (i & 7U)))
      s = SOFT_MAX - s;

    // Deinterleaving: same table as before, position = byte * 8 + bit
    soft[INTERLEAVE_TABLE_RX[i * 2U] * 8U + INTERLEAVE_TABLE_RX[i * 2U + 1U]] = uint8_t(s);
  }

  for (uint8_t k = 0U; k < 4U; k++)
    m_pathMetric[k] = 0;

  int decodeData[2U];

  m_mar = 0U;
  for (uint16_t i = 0U; i < 660U; i += 2U) {
    decodeData[1U] = soft[i];
    decodeData[0U] = soft[i + 1U];

    viterbiDecode(decodeData);
  }

  traceBack();
  if (adj == 0)
    m_headerMetric = m_pathMetric[0U];

  for (int i = 0; i < int(DSTAR_HEADER_LENGTH_BYTES); i++)
    out[i] = 0x00U;

  unsigned int j = 0;
  for (int i = 329; i >= 0; i--) {
    if (READ_BIT1(m_fecOutput, i))
      out[j >> 3] |= (0x01U << (j & 7));

    j++;
  }

  return checksum(out);
}

void CDStarRX::acs(int* metric)
{
  int tempMetric[4U];

  unsigned int j = m_mar >> 3;
  unsigned int k = m_mar & 7;

  // Pres. state = S0, Prev. state = S0 & S2
  int m1 = metric[0U] + m_pathMetric[0U];
  int m2 = metric[4U] + m_pathMetric[2U];
  tempMetric[0U] = m1 < m2 ? m1 : m2;
  if (m1 < m2)
    m_pathMemory0[j] &= BIT_MASK_TABLE0[k];
  else
    m_pathMemory0[j] |= BIT_MASK_TABLE1[k];

  // Pres. state = S1, Prev. state = S0 & S2
  m1 = metric[1U] + m_pathMetric[0U];
  m2 = metric[5U] + m_pathMetric[2U];
  tempMetric[1U] = m1 < m2 ? m1 : m2;
  if (m1 < m2)
    m_pathMemory1[j] &= BIT_MASK_TABLE0[k];
  else
    m_pathMemory1[j] |= BIT_MASK_TABLE1[k];

  // Pres. state = S2, Prev. state = S2 & S3
  m1 = metric[2U] + m_pathMetric[1U];
  m2 = metric[6U] + m_pathMetric[3U];
  tempMetric[2U] = m1 < m2 ? m1 : m2;
  if (m1 < m2)
    m_pathMemory2[j] &= BIT_MASK_TABLE0[k];
  else
    m_pathMemory2[j] |= BIT_MASK_TABLE1[k];

  // Pres. state = S3, Prev. state = S1 & S3
  m1 = metric[3U] + m_pathMetric[1U];
  m2 = metric[7U] + m_pathMetric[3U];
  tempMetric[3U] = m1 < m2 ? m1 : m2;
  if (m1 < m2)
    m_pathMemory3[j] &= BIT_MASK_TABLE0[k];
  else
    m_pathMemory3[j] |= BIT_MASK_TABLE1[k];

  for (unsigned int i = 0U; i < 4U; i++)
    m_pathMetric[i] = tempMetric[i];

  m_mar++;
}
 
void CDStarRX::viterbiDecode(const int* data)
{
  int metric[8U];

  // Cost of an expected 0 bit = d, of an expected 1 bit = SOFT_MAX - d
  const int d1  = data[1U];
  const int d0  = data[0U];
  const int d1n = SOFT_MAX - d1;
  const int d0n = SOFT_MAX - d0;

  metric[0] = d1  + d0;     // expected (0,0)
  metric[1] = d1n + d0n;    // expected (1,1)
  metric[2] = d1n + d0;     // expected (1,0)
  metric[3] = d1  + d0n;    // expected (0,1)
  metric[4] = d1n + d0n;    // expected (1,1)
  metric[5] = d1  + d0;     // expected (0,0)
  metric[6] = d1  + d0n;    // expected (0,1)
  metric[7] = d1n + d0;     // expected (1,0)

  acs(metric);
}

void CDStarRX::traceBack()
{
  // Start from the S0, t=31
  unsigned int j = 0U;
  unsigned int k = 0U;
  for (int i = 329; i >= 0; i--) {
    switch (j) {
      case 0U: // if state = S0
        if (!READ_BIT1(m_pathMemory0, i))
          j = 0U;
        else
          j = 2U;
        WRITE_BIT1(m_fecOutput, k, false);
        k++;
        break;


      case 1U: // if state = S1
        if (!READ_BIT1(m_pathMemory1, i))
          j = 0U;
        else
          j = 2U;
        WRITE_BIT1(m_fecOutput, k, true);
        k++;
        break;

      case 2U: // if state = S1
        if (!READ_BIT1(m_pathMemory2, i))
          j = 1U;
        else
          j = 3U;
        WRITE_BIT1(m_fecOutput, k, false);
        k++;
        break;

      case 3U: // if state = S1
        if (!READ_BIT1(m_pathMemory3, i))
          j = 1U;
        else
          j = 3U;
        WRITE_BIT1(m_fecOutput, k, true);
        k++;
        break;
    }
  }
}

bool CDStarRX::checksum(const uint8_t* header) const
{
  union {
    uint16_t crc16;
    uint8_t  crc8[2U];
  };

  crc16 = 0xFFFFU;
  for (uint8_t i = 0U; i < (DSTAR_HEADER_LENGTH_BYTES - 2U); i++)
    crc16 = uint16_t(crc8[1U]) ^ CCITT_TABLE[crc8[0U] ^ header[i]];

  crc16 = ~crc16;

  return crc8[0U] == header[DSTAR_HEADER_LENGTH_BYTES - 2U] && crc8[1U] == header[DSTAR_HEADER_LENGTH_BYTES - 1U];
}

#endif

