/*
 *   Copyright (C) 2009-2017,2020,2024 by Jonathan Naylor G4KLX
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

#if defined(MODE_YSF)

#include "Globals.h"
#include "YSFRX.h"
#include "Utils.h"

const uint8_t MAX_SYNC_BIT_START_ERRS = 2U;
const uint8_t MAX_SYNC_BIT_RUN_ERRS   = 4U;

const uint8_t MAX_SYNC_SYMBOLS_ERRS = 3U;

const uint8_t BIT_MASK_TABLE[] = {0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U};

#define WRITE_BIT1(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE[(i)&7]) : (p[(i)>>3] & ~BIT_MASK_TABLE[(i)&7])

const uint8_t NOAVEPTR = 99U;

const uint16_t NOENDPTR = 9999U;

const unsigned int MAX_SYNC_FRAMES = 4U + 1U;

CYSFRX::CYSFRX() :
m_state(YSFRXS_NONE),
m_bitBuffer(),
m_buffer(),
m_bitPtr(0U),
m_dataPtr(0U),
m_startPtr(NOENDPTR),
m_endPtr(NOENDPTR),
m_syncPtr(NOENDPTR),
m_minSyncPtr(NOENDPTR),
m_maxSyncPtr(NOENDPTR),
m_maxCorr(0),
m_lostCount(0U),
m_countdown(0U),
m_centreVal(0),
m_thresholdVal(0),
m_averagePtr(NOAVEPTR),
m_rssiAccum(0U),
m_rssiCount(0U),
m_delta(0)
{
}

void CYSFRX::reset()
{
  m_state        = YSFRXS_NONE;
  m_dataPtr      = 0U;
  m_bitPtr       = 0U;
  m_maxCorr      = 0;
  m_averagePtr   = NOAVEPTR;
  m_startPtr     = NOENDPTR;
  m_endPtr       = NOENDPTR;
  m_syncPtr      = NOENDPTR;
  m_minSyncPtr   = NOENDPTR;
  m_maxSyncPtr   = NOENDPTR;
  m_centreVal    = 0;
  m_thresholdVal = 0;
  m_lostCount    = 0U;
  m_countdown    = 0U;
  m_rssiAccum    = 0U;
  m_rssiCount    = 0U;
  m_delta        = 0;
}

void CYSFRX::samples(const q15_t* samples, uint16_t* rssi, uint8_t length)
{
  for (uint8_t i = 0U; i < length; i++) {
    q15_t sample = samples[i];

    m_rssiAccum += rssi[i];
    m_rssiCount++;

    m_bitBuffer[m_bitPtr] <<= 1;
    if (sample < 0)
      m_bitBuffer[m_bitPtr] |= 0x01U;

    m_buffer[m_dataPtr] = sample;

    switch (m_state) {
    case YSFRXS_DATA:
      processData(sample);
      break;
    default:
      processNone(sample);
      break;
    }

    m_dataPtr++;
    if (m_dataPtr >= YSF_FRAME_LENGTH_SAMPLES)
      m_dataPtr = 0U;

    m_bitPtr++;
    if (m_bitPtr >= YSF_RADIO_SYMBOL_LENGTH)
      m_bitPtr = 0U;
  }
}

void CYSFRX::processNone(q15_t sample)
{
  bool ret = correlateSync();
  if (ret) {
    // On the first sync, start the countdown to the state change
    if (m_countdown == 0U) {
      m_rssiAccum = 0U;
      m_rssiCount = 0U;

      io.setDecode(true);
      io.setADCDetection(true);

      m_averagePtr = NOAVEPTR;

      m_countdown = 5U;
    }
  }

  if (m_countdown > 0U)
    m_countdown--;

  if (m_countdown == 1U) {
    m_minSyncPtr = m_syncPtr + YSF_FRAME_LENGTH_SAMPLES - 1U;
    if (m_minSyncPtr >= YSF_FRAME_LENGTH_SAMPLES)
      m_minSyncPtr -= YSF_FRAME_LENGTH_SAMPLES;

    m_maxSyncPtr = m_syncPtr + 1U;
    if (m_maxSyncPtr >= YSF_FRAME_LENGTH_SAMPLES)
      m_maxSyncPtr -= YSF_FRAME_LENGTH_SAMPLES;

    m_state      = YSFRXS_DATA;
    m_countdown  = 0U;
  }
}

void CYSFRX::processData(q15_t sample)
{
  if (m_minSyncPtr < m_maxSyncPtr) {
    if (m_dataPtr >= m_minSyncPtr && m_dataPtr <= m_maxSyncPtr)
      correlateSync();
  } else {
    if (m_dataPtr >= m_minSyncPtr || m_dataPtr <= m_maxSyncPtr)
      correlateSync();
  }

  if (m_dataPtr == m_endPtr) {
    // Only update the position, centre and threshold if they are from a good sync
    if (m_lostCount == MAX_SYNC_FRAMES) {
      alignSync();
      syncLevels();

      m_minSyncPtr = m_syncPtr + YSF_FRAME_LENGTH_SAMPLES - 1U;
      if (m_minSyncPtr >= YSF_FRAME_LENGTH_SAMPLES)
        m_minSyncPtr -= YSF_FRAME_LENGTH_SAMPLES;

      m_maxSyncPtr = m_syncPtr + 1U;
      if (m_maxSyncPtr >= YSF_FRAME_LENGTH_SAMPLES)
        m_maxSyncPtr -= YSF_FRAME_LENGTH_SAMPLES;
    }

    DEBUG4("YSFRX: sync found pos/centre/threshold", m_syncPtr, m_centreVal, m_thresholdVal);

    uint8_t frame[YSF_FRAME_LENGTH_BYTES + 3U];
    samplesToBits(m_startPtr, YSF_FRAME_LENGTH_SYMBOLS, frame, 8U, m_centreVal, m_thresholdVal, m_delta);

    // We've not seen a data sync for too long, signal RXLOST and change to RX_NONE
    m_lostCount--;
    if (m_lostCount == 0U) {
      DEBUG1("YSFRX: sync timed out, lost lock");

      io.setDecode(false);
      io.setADCDetection(false);

      serial.writeYSFLost();

      m_state      = YSFRXS_NONE;
      m_endPtr     = NOENDPTR;
      m_averagePtr = NOAVEPTR;
      m_countdown  = 0U;
      m_maxCorr    = 0;
    } else {
      frame[0U] = m_lostCount == (MAX_SYNC_FRAMES - 1U) ? 0x01U : 0x00U;
      writeRSSIData(frame);
      m_maxCorr = 0;
    }
  }
}

bool CYSFRX::correlateSync()
{
  if (countBits32((m_bitBuffer[m_bitPtr] & YSF_SYNC_SYMBOLS_MASK) ^ YSF_SYNC_SYMBOLS) <= MAX_SYNC_SYMBOLS_ERRS) {
    uint16_t ptr = m_dataPtr + YSF_FRAME_LENGTH_SAMPLES - YSF_SYNC_LENGTH_SAMPLES + YSF_RADIO_SYMBOL_LENGTH;
    if (ptr >= YSF_FRAME_LENGTH_SAMPLES)
      ptr -= YSF_FRAME_LENGTH_SAMPLES;

    q31_t corr = 0;
    q31_t sumL  = 0;   // sum of the levels
    q31_t sumLL = 0;   // sum of the levels squared
    q31_t sumS  = 0;   // sum of the samples
    q31_t sumLS = 0;   // sum of level * sample

    for (uint8_t i = 0U; i < YSF_SYNC_LENGTH_SYMBOLS; i++) {
      q15_t val = m_buffer[ptr];

      q31_t level = -YSF_SYNC_SYMBOLS_VALUES[i];   // the received samples have the opposite sign of the symbol values
      sumL  += level;
      sumLL += level * level;
      sumS  += val;
      sumLS += level * val;

      switch (YSF_SYNC_SYMBOLS_VALUES[i]) {
      case +3:
        corr -= (val + val + val);
        break;
      case +1:
        corr -= val;
        break;
      case -1:
        corr += val;
        break;
      default:  // -3
        corr += (val + val + val);
        break;
      }

      ptr += YSF_RADIO_SYMBOL_LENGTH;
      if (ptr >= YSF_FRAME_LENGTH_SAMPLES)
        ptr -= YSF_FRAME_LENGTH_SAMPLES;
    }

    if (corr > m_maxCorr) {
      if (m_averagePtr == NOAVEPTR) {
        // Centre and threshold from the known sync symbols
        const q31_t n = YSF_SYNC_LENGTH_SYMBOLS;

        q31_t den = n * sumLL - sumL * sumL;
        q31_t num = n * sumLS - sumL * sumS;
        if (den <= 0 || num <= 0)
          return false;

        q31_t gain = num / den;

        m_centreVal    = q15_t((sumS - gain * sumL) / n);
        m_thresholdVal = q15_t((2 * num) / den);
      }

      uint16_t startPtr = m_dataPtr + YSF_FRAME_LENGTH_SAMPLES - YSF_SYNC_LENGTH_SAMPLES + YSF_RADIO_SYMBOL_LENGTH;
      if (startPtr >= YSF_FRAME_LENGTH_SAMPLES)
        startPtr -= YSF_FRAME_LENGTH_SAMPLES;

      uint8_t sync[YSF_SYNC_BYTES_LENGTH];
      samplesToBits(startPtr, YSF_SYNC_LENGTH_SYMBOLS, sync, 0U, m_centreVal, m_thresholdVal);

      uint8_t maxErrs;
      if (m_state == YSFRXS_NONE)
        maxErrs = MAX_SYNC_BIT_START_ERRS;
      else
        maxErrs = MAX_SYNC_BIT_RUN_ERRS;

      uint8_t errs = 0U;
      for (uint8_t i = 0U; i < YSF_SYNC_BYTES_LENGTH; i++)
        errs += countBits8(sync[i] ^ YSF_SYNC_BYTES[i]);

      if (errs <= maxErrs) {
        m_maxCorr   = corr;
        m_lostCount = MAX_SYNC_FRAMES;
        m_syncPtr   = m_dataPtr;

        m_startPtr = startPtr;

        // Two samples after the last symbol of the frame, so that it can be interpolated
        m_endPtr = m_dataPtr + YSF_FRAME_LENGTH_SAMPLES - YSF_SYNC_LENGTH_SAMPLES + 1U;
        if (m_endPtr >= YSF_FRAME_LENGTH_SAMPLES)
          m_endPtr -= YSF_FRAME_LENGTH_SAMPLES;

        return true;
      }
    }
  }

  return false;
}

q15_t CYSFRX::sampleAt(uint16_t pos, int16_t delta) const
{
  if (delta == 0)
    return m_buffer[pos];

  uint16_t prev = (pos == 0U) ? YSF_FRAME_LENGTH_SAMPLES - 1U : pos - 1U;
  uint16_t next = (pos + 1U >= YSF_FRAME_LENGTH_SAMPLES) ? 0U : pos + 1U;

  q31_t s0 = m_buffer[pos];
  q31_t sm = m_buffer[prev];
  q31_t sp = m_buffer[next];

  q31_t d = delta;
  q31_t v = s0 + ((d * (sp - sm)) >> 9) + (((d * d) * (sp - 2 * s0 + sm)) >> 17);

  if (v > 32767)
    v = 32767;
  if (v < -32768)
    v = -32768;

  return q15_t(v);
}

// Correlation of the sync symbols with the buffer, for a sync position shifted by offset samples
q31_t CYSFRX::syncCorrelation(int8_t offset) const
{
  int32_t ptr = int32_t(m_syncPtr) + int32_t(YSF_FRAME_LENGTH_SAMPLES) - int32_t(YSF_SYNC_LENGTH_SAMPLES) + int32_t(YSF_RADIO_SYMBOL_LENGTH) + offset;
  ptr %= int32_t(YSF_FRAME_LENGTH_SAMPLES);

  q31_t corr = 0;
  for (uint8_t i = 0U; i < YSF_SYNC_LENGTH_SYMBOLS; i++) {
    q15_t val = m_buffer[ptr];

    switch (YSF_SYNC_SYMBOLS_VALUES[i]) {
    case +3:
      corr -= (val + val + val);
      break;
    case +1:
      corr -= val;
      break;
    case -1:
      corr += val;
      break;
    default:  // -3
      corr += (val + val + val);
      break;
    }

    ptr += YSF_RADIO_SYMBOL_LENGTH;
    if (ptr >= int32_t(YSF_FRAME_LENGTH_SAMPLES))
      ptr -= int32_t(YSF_FRAME_LENGTH_SAMPLES);
  }

  return corr;
}

// Find the best sync position to the nearest sample, and the sub-sample position of the correlation peak
void CYSFRX::alignSync()
{
  q31_t c[5U];
  for (int8_t o = -2; o <= 2; o++)
    c[o + 2] = syncCorrelation(o);

  uint8_t best = 2U;
  for (uint8_t i = 0U; i < 5U; i++) {
    if (c[i] > c[best])
      best = i;
  }

  int8_t shift = int8_t(best) - 2;
  if (shift != 0) {
    m_syncPtr  = uint16_t((int32_t(m_syncPtr)  + shift + int32_t(YSF_FRAME_LENGTH_SAMPLES)) % int32_t(YSF_FRAME_LENGTH_SAMPLES));
    m_startPtr = uint16_t((int32_t(m_startPtr) + shift + int32_t(YSF_FRAME_LENGTH_SAMPLES)) % int32_t(YSF_FRAME_LENGTH_SAMPLES));
  }

  // Sub-sample position (1/256 sample units, -128..+128) from a parabola through the three correlations around the peak
  m_delta = 0;
  if (best >= 1U && best <= 3U) {
    q31_t den = c[best - 1U] - 2 * c[best] + c[best + 1U];
    if (den < 0) {
      q31_t p = ((c[best - 1U] - c[best + 1U]) << 7) / den;
      if (p > 128)
        p = 128;
      if (p < -128)
        p = -128;
      m_delta = int16_t(p);
    }
  }
}

// Centre and threshold from the known sync symbols, by a least squares fit of
// sample = centre + gain * level over the (interpolated) sync samples. The threshold is
// halfway between the inner (1) and outer (3) levels, i.e. twice the gain.
void CYSFRX::syncLevels()
{
  uint16_t ptr = m_startPtr;

  q31_t sumL  = 0;   // sum of the levels
  q31_t sumLL = 0;   // sum of the levels squared
  q31_t sumS  = 0;   // sum of the samples
  q31_t sumLS = 0;   // sum of level * sample

  for (uint8_t i = 0U; i < YSF_SYNC_LENGTH_SYMBOLS; i++) {
    q31_t sample = sampleAt(ptr, m_delta);
    q31_t level  = -YSF_SYNC_SYMBOLS_VALUES[i];   // the received samples have the opposite sign of the symbol values

    sumL  += level;
    sumLL += level * level;
    sumS  += sample;
    sumLS += level * sample;

    ptr += YSF_RADIO_SYMBOL_LENGTH;
    if (ptr >= YSF_FRAME_LENGTH_SAMPLES)
      ptr -= YSF_FRAME_LENGTH_SAMPLES;
  }

  const q31_t n = YSF_SYNC_LENGTH_SYMBOLS;

  q31_t den = n * sumLL - sumL * sumL;
  q31_t num = n * sumLS - sumL * sumS;      // the gain is num / den
  if (den <= 0 || num <= 0)
    return;

  q31_t gain = num / den;

  m_centreVal    = q15_t((sumS - gain * sumL) / n);
  m_thresholdVal = q15_t((2 * num) / den);

  m_averagePtr = 0U;
}

void CYSFRX::samplesToBits(uint16_t start, uint16_t count, uint8_t* buffer, uint16_t offset, q15_t centre, q15_t threshold, int16_t delta)
{
  for (uint16_t i = 0U; i < count; i++) {
    q15_t sample = sampleAt(start, delta) - centre;

    if (sample < -threshold) {
      WRITE_BIT1(buffer, offset, false);
      offset++;
      WRITE_BIT1(buffer, offset, true);
      offset++;
    } else if (sample < 0) {
      WRITE_BIT1(buffer, offset, false);
      offset++;
      WRITE_BIT1(buffer, offset, false);
      offset++;
    } else if (sample < threshold) {
      WRITE_BIT1(buffer, offset, true);
      offset++;
      WRITE_BIT1(buffer, offset, false);
      offset++;
    } else {
      WRITE_BIT1(buffer, offset, true);
      offset++;
      WRITE_BIT1(buffer, offset, true);
      offset++;
    }

    start += YSF_RADIO_SYMBOL_LENGTH;
    if (start >= YSF_FRAME_LENGTH_SAMPLES)
      start -= YSF_FRAME_LENGTH_SAMPLES;
  }
}

void CYSFRX::writeRSSIData(uint8_t* data)
{
#if defined(SEND_RSSI_DATA)
  if (m_rssiCount > 0U) {
    uint16_t rssi = m_rssiAccum / m_rssiCount;

    data[121U] = (rssi >> 8) & 0xFFU;
    data[122U] = (rssi >> 0) & 0xFFU;

    serial.writeYSFData(data, YSF_FRAME_LENGTH_BYTES + 3U);
  } else {
    serial.writeYSFData(data, YSF_FRAME_LENGTH_BYTES + 1U);
  }
#else
  serial.writeYSFData(data, YSF_FRAME_LENGTH_BYTES + 1U);
#endif

  m_rssiAccum = 0U;
  m_rssiCount = 0U;
}

#endif

