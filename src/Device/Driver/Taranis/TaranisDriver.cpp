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

#include "Device/Driver/Taranis/Internal.hpp"
#include "Device/Driver/Taranis/TaranisDriver.hpp"
#include "LogFile.hpp"

#include "Taranis.h"



static Device *
TaranisCreateOnPort(const DeviceConfig &config, Port &com_port)
{
  return new TaranisDevice();
}

const struct DeviceRegister taranis_driver = {
  _T("Taranis"),
  _T("Taranis"),
  DeviceRegister::RAW_GPS_DATA,
  TaranisCreateOnPort,
};


bool
TaranisDevice::DataReceived(const void *_data, size_t length,
                          struct NMEAInfo &info){
  const char *data = (const char *)_data;
  for (int count = 0; count < (int)length; count++)
  {
    Taranis.parse(data[count], info);
  }

  return true;
}


