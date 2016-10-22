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

#ifndef TARANIS_HPP
#define TARANIS_HPP

#include "stdint.h"

#define SENSOR_NO_DATA_ID           0x0000
#define FRSKY_TELEMETRY_START_FRAME 0x7E
#define FRSKY_SENSOR_DATA_FRAME     0x10
#define FRSKY_STUFFING              0x7D




class cTaranis
{
public:
  void parse(uint8_t c, struct NMEAInfo &info);
private:
#define XJT_DEFAULT_ID ID25

#define XJT_RSSI_DATA_ID          0xF101
#define XJT_ADC1_DATA_ID          0xF102
#define XJT_ADC2_DATA_ID          0xF103
#define XJT_RXBATT_DATA_ID        0xF104
#define XJT_SWR_DATA_ID           0xF105
#define XJT_FAS_VOLTAGE_DATA_ID   0x003B
#define XJT_FAS_CURRENT_DATA_ID   0x0028
#define XJT_FGS_FUEL_DATA_ID      0x0004
#define XJT_FLVS_VOLTAGE_DATA_ID  0x0006
#define XJT_FVAS_ALTITUDE_DATA_ID 0x0021
#define XJT_FVAS_VSI_DATA_ID      0x0030 // Not documented in FrSky spec, added based on OpenTX sources.
#define XJT_GPS_LAT_DATA_ID       0x0023
#define XJT_GPS_LON_DATA_ID       0x0022
#define XJT_GPS_ALTITUDE_DATA_ID  0x0009
#define XJT_GPS_SPEED_DATA_ID     0x0019
#define XJT_GPS_COG_DATA_ID       0x001C
#define XJT_GPS_DATE_DATA_ID      0x0016
#define XJT_GPS_TIME_DATA_ID      0x0018
#define XJT_TAS_ACCX_DATA_ID      0x0024
#define XJT_TAS_ACCY_DATA_ID      0x0025
#define XJT_TAS_ACCZ_DATA_ID      0x0026
#define XJT_TEMS_T1_DATA_ID       0x0002
#define XJT_TEMS_T2_DATA_ID       0x0005
#define XJT_RPMS_RPM_DATA_ID      0x0003

#define VARIO_ALT_DATA_ID         0x0100
#define VARIO_VSI_DATA_ID         0x0110


#define GPS_LAT_LON_DATA_ID       0x0800
#define GPS_ALT_DATA_ID           0x0820
#define GPS_SPEED_DATA_ID         0x0830
#define GPS_COG_DATA_ID           0x0840
#define GPS_DATE_TIME_DATA_ID     0x0850

#define FCS_CURR_DATA_ID          0x0200
#define FCS_VOLT_DATA_ID          0x0210

#define RPM_T1_DATA_ID            0x0400
#define RPM_T2_DATA_ID            0x0410
#define RPM_ROT_DATA_ID           0x0500

#define ACC_X                     0x0700
#define ACC_Y                     0x0710
#define ACC_Z                     0x0720

#define FR_ID_FUEL                0x0600





  void processvalue(uint16_t SensorId, uint32_t value, struct NMEAInfo &info);



  enum State {
    START_FRAME,
    SENSOR_ID,
    DATA_FRAME,
    APP_ID_BYTE_1,
    APP_ID_BYTE_2,
    DATA_BYTE_1,
    DATA_BYTE_2,
    DATA_BYTE_3,
    DATA_BYTE_4,
    CRC};

  State     state;
  bool      hasStuffing;
  uint8_t   id;
  uint16_t  appId;
  uint32_t  data;
  uint16_t  crc;


  // values read from Telemetry

  float lat;
  float lon;
  float altitude;
  float speed;
  float cog;

  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  double last_time;


};




#endif
