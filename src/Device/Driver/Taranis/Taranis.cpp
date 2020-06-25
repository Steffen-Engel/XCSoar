/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "stdint.h"
#include "LogFile.hpp"
#include "NMEA/Info.hpp"
#include "IO/TextWriter.hpp"
#include "Device/Parser.hpp"


#include "Taranis.h"

uint32_t crcCounter = 0;

void
cTaranis::processvalue(uint16_t SensorId, uint32_t value, struct NMEAInfo &info)
{
  GeoPoint location;

  // we are alive, heartbeat
  info.alive.Update(info.clock);

  BrokenDateTime now;
  now = BrokenDateTime::NowUTC();

  double this_time;
  this_time = double(now.hour*60*60 + now.minute*60 + now.second);

  if (NMEAParser::TimeHasAdvanced(this_time, last_time, info))
  {
    // always deliver system time
    info.date_time_utc = now;
    info.time_available.Update(info.clock);
  }

  switch (SensorId) {
  case XJT_RSSI_DATA_ID:
    //LogDebug(_T("RSSI is %5lddB, "), (long int)value);
    break;
  case XJT_SWR_DATA_ID:
    // LogDebug(_T("SWR is senseless %8ld, "), (long int)value);
    break;
  case VARIO_ALT_DATA_ID:
    //LogDebug(_T("altitude is %7.1f, "), 0.01 * ((int32_t)value));
    info.ProvideBaroAltitudeTrue(0.01 * ((int32_t)value));
    break;
  case VARIO_VSI_DATA_ID:
    //LogDebug(_T("variometer is %7.2f, "), 0.01 * ((int32_t)value));
    info.ProvideNoncompVario(0.01 * ((int32_t)value));
    break;

  case GPS_LAT_LON_DATA_ID:
    //LogDebug(_T("GPS-latlon %08lx, "), (long int )value);

    // parse data and prepare for delivery
    float latLonData;
    latLonData = (value & 0x3FFFFFFF) / 10000.0 / 60.0;
    if ((value & 0x40000000) > 0) {
      latLonData = -latLonData;                 // is negative?
    }
    if ((value & 0x80000000) == 0) {
      lat = latLonData;
      //LogDebug(_T("lat %f, "), lat);
    } else {
      lon = latLonData; // is latitude?
      //LogDebug(_T("lon %f, "), lon);
    }

    // LogDebug(_T("thistime %f, lat %f, lon %f"), this_time, lat, lon);
    // deliver position
    location.latitude = Angle::Degrees(lat);
    location.longitude = Angle::Degrees(lon);
    info.location = location;
    info.gps.real = true;

    info.location_available.Update(info.clock);

    break;
  case GPS_ALT_DATA_ID:
    //LogDebug(_T("GPS-alt %5.2fm, "),  0.01*value);  // in cm
    altitude = 0.01 * value;
    info.gps_altitude_available.Update(info.clock);
    info.gps_altitude = altitude;
    break;
  case GPS_SPEED_DATA_ID:
    //LogDebug(_T("GPS-speed %5.2fkm/h, "),  1.0/1944.0*value); // knots to km/h
    speed = 1.0 / 1944.0 * value / 3.6; // knots to ms
    info.ground_speed = speed;
    info.ground_speed_available.Update(info.clock);
    break;
  case GPS_COG_DATA_ID:
    //LogDebug(_T("GPS-cog %5.2f° "),  0.01*(long int)value);
    cog = 0.01 * value;
    info.track = Angle::Degrees(cog);
    info.track_available.Update(info.clock);
    break;
  case GPS_DATE_TIME_DATA_ID:
    //LogDebug(_T("GPS-datetime %08lx, "),  (long int)value);

    if ((value & 0xFF) > 0){  // is date?
      value >>= 8;
      day = value & 0xFF;
      value >>= 8;
      month = value & 0xFF;
      value >>= 8;
      year = 2000 + (value & 0xFF);
      //LogDebug(_T("GPS-Date %02d.%02d.%04d, "),  day, month, year);
    } else {
      value >>= 8;
      second = value & 0xFF;
      value >>= 8;
      minute = value & 0xFF;
      value >>= 8;
      hour = value & 0xFF;
      //LogDebug(_T("GPS-Time %02d:%02d:%02d, "),  hour, minute, second);
    }
    // deliver time
    if (year != 0)
    {
/* not too good, we deliver system time as pseudo GPS-time to have continous data available...
      info.date_time_utc.day = day;
      info.date_time_utc.month = month;
      info.date_time_utc.year = year;
      info.date_time_utc.hour = hour;
      info.date_time_utc.minute = minute;
      info.date_time_utc.second = second;
      info.time_available.Update(info.clock);
*/
    }
    break;
  case FCS_CURR_DATA_ID:
    //LogDebug(_T("Current %5.1fA"),  0.1*(long int)value);
    break;
  case FCS_VOLT_DATA_ID:
    // LogDebug(_T("Voltage %5.2f"),  0.01*(long int)value);
    info.voltage = 0.01*(long int)value;
    info.voltage_available.Update(info.clock);

    break;
  case RPM_ROT_DATA_ID:
    //LogDebug(_T("rpm %5d"),  (int32_t)value);
    break;
  case ACC_X:
    //LogDebug(_T("acc x %5.2fg"),  0.01*(int32_t)value);
    break;
  case ACC_Y:
    //LogDebug(_T("acc y %5.2fg"),  0.01*(int32_t)value);
    break;
  case ACC_Z:
    //LogDebug(_T("acc z %5.2fg"),  0.01*(int32_t)value);
    info.acceleration.ProvideGLoad(0.01*value, true);
    break;
  case FR_ID_FUEL:
    //LogDebug(_T("Fuel %5d"),  (int32_t)value);
    info.battery_level = 1.0*(long int)value;
    info.battery_level_available.Update(info.clock);
    break;
  default:
    LogDebug(_T("unknown id 0x%04x value 0x%08lx  %ld"), SensorId,
             (long unsigned int)value, (long unsigned int)value);
    break;
  }
}


void cTaranis::parse(uint8_t c, struct NMEAInfo &info)
{

  uint8_t byte = c;
  if (byte == FRSKY_TELEMETRY_START_FRAME) {
    state = SENSOR_ID;
  }       // Regardles of the state restart state machine when start frame found
  else {
    if (hasStuffing == true) {
      byte ^= 0x20;
      hasStuffing = false;
    }                             // Xor next byte with 0x20 to remove stuffing
    if ((byte == FRSKY_STUFFING) && (state > DATA_FRAME) && (state <= CRC)) {
//          LogDebug(_T("start stuffing"));
      hasStuffing = true; // Skip stuffing byte in data and mark to xor next byte with 0x20
    } else if (state == SENSOR_ID) {
      id = byte;
      state = DATA_FRAME;
    }                      // Store the sensor ID, start sarching for data frame
    else if ((state == DATA_FRAME) && (byte == FRSKY_SENSOR_DATA_FRAME)) {
      crc = byte; // Data frame found, initialize the CRC and start collecting APP ID
      state = APP_ID_BYTE_1;
    } else if (state == APP_ID_BYTE_1) {
      ((uint8_t*)&appId)[0] = byte;
      state = APP_ID_BYTE_2;
    }   // APP ID first byte collected, look for second byte
    else if (state == APP_ID_BYTE_2) {
      ((uint8_t*)&appId)[1] = byte;
      state = DATA_BYTE_1;
    }   // APP ID second byte collected, store APP ID and start looking for DATA
    else if (state == DATA_BYTE_1) {
      ((uint8_t*)&data)[0] = byte;
      state = DATA_BYTE_2;
    }        // DATA first byte collected, look for second byte
    else if (state == DATA_BYTE_2) {
      ((uint8_t*)&data)[1] = byte;
      state = DATA_BYTE_3;
    }        // DATA second byte collected, look for third byte
    else if (state == DATA_BYTE_3) {
      ((uint8_t*)&data)[2] = byte;
      state = DATA_BYTE_4;
    }        // DATA third byte collected, look for fourth byte
    else if (state == DATA_BYTE_4) {
      ((uint8_t*)&data)[3] = byte;
      state = CRC;
    }                // DATA fourth byte collected, store DATA and look for CRC
    else if (state == CRC)        // read CRC and compare with calculated one.
    { // If OK, send data to registered sensors for decoding and restart the state machine.
      crc += byte;
      crc += crc >> 8;
      crc &= 0x00ff;
      if (crc == 0xFF) {
        //LogDebug(_T("ID is %02x, appId %04x, value %ld crc %04x"), id, appId, (long int)data, crc);
        processvalue(appId, data, info);
      } else {
        crcCounter++;
        LogDebug(
            _T("CRC wrong: 0x10,%02x,%02x,%02x,%02x,%02x,%02x,%02x CRC:%04x"),
            appId >> 8, appId & 0xff, (data >> 24) & 0xff, (data >> 16) & 0xff,
            (data >> 8) & 0xff, data & 0xff, byte, crc);
        if (hasStuffing) {
          //LogDebug(_T("has stuffing"));
        }
      }
      hasStuffing = false;
      state = START_FRAME;
    } else {
      hasStuffing = false;
      state = START_FRAME;
    }

    // Update CRC value
    if ((state > APP_ID_BYTE_1) && (state <= CRC) && (hasStuffing == false)) {
      crc += byte;
      crc += crc >> 8;
      crc &= 0x00ff;
    }
  }

}

