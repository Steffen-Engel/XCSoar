/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2013 The XCSoar Project
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

#include "Device/Driver/yAll/Internal.hpp"
#include "Device/Driver/yAll/yAllDriver.hpp"
#include "Device/Driver.hpp"
#include "Device/Port/Port.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/Derived.hpp"
#include "NMEA/InputLine.hpp"
#include "Atmosphere/Temperature.hpp"
#include "LogFile.hpp"
#include "InfoBoxes/InfoBoxManager.hpp"

#include "Operation/Operation.hpp"
#include "yAllProtocol.h"
#include "yAll.h"

#include <tchar.h>

#include <stdint.h>

#define DATA_TIME 50






bool
yAllDevice::DataReceived([[maybe_unused]] std::span<const std::byte> s,
                         [[maybe_unused]] struct NMEAInfo &info) noexcept {
//  LogDebug(_T("yAll data %d"), (int)length);

  [[maybe_unused]] const char *data = (const char *)s.data();
  for (int count = 0; count < (int)s.size(); count++)
  {
    yAll.parse(data[count], info);
  }

  return true;
}


bool
yAllDevice::EnableNMEA([[maybe_unused]]OperationEnvironment &env)
{
  LogDebug(_T("yAll EnableNMEA"));

  //env.Sleep(5000);
  //yAll.sendRequest(MSP_RESET);
  //env.Sleep(500);
  //yAll.sendRequest(MSP_LOGGER_SET, (uint32_t)DATA_TIME);
  yAll.sendRequest(MSP_IDENT);

  return true;
}


void
yAllDevice::OnSysTicker(){
  //LogDebug(_T("yAll SysTicker"));
  yAll.sendRequest(MSP_LOGGER_SET, (uint32_t)DATA_TIME);
}


void
yAllDevice::LinkTimeout(){
  LogDebug(_T("yAll LinkTimeout"));
  yAll.sendRequest(MSP_LOGGER_SET, (uint32_t)DATA_TIME);
}


void
yAllDevice::GetInfo(tIdent &Ident)
{
  Ident = yAll.Ident;
};


void
yAllDevice::Restart()
{
  yAll.sendRequest(MSP_RESET);
}


void
yAllDevice::CalibrateAcc()
{
  yAll.sendRequest(MSP_ACC_CALIBRATION);
}


void
yAllDevice::CalibrateIAS()
{
  yAll.sendRequest(MSP_IAS_CALIBRATION);
}


void
yAllDevice::CalibrateMag()
{
  yAll.sendRequest(MSP_MAG_CALIBRATION);
}


void
yAllDevice::OnCalculatedUpdate([[maybe_unused]]const MoreData &basic,
                        const DerivedInfo &calculated)
{
  // we need a copy of the flightstate...
  yAll.FlightState = calculated.flight.flying;
}



static Device *
yAllCreateOnPort([[maybe_unused]]const DeviceConfig &config, Port &com_port)
{
  return new yAllDevice(com_port);
}

const struct DeviceRegister yall_driver = {
  _T("yAll"),
  _T("yAll"),
  DeviceRegister::RAW_GPS_DATA | DeviceRegister::MANAGE,
  yAllCreateOnPort,
};
