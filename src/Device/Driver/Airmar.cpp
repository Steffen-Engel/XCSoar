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

#include "Device/Driver/Airmar.hpp"
#include "Device/Driver.hpp"
#include "Device/Port/Port.hpp"
#include "NMEA/InputLine.hpp"
#include "NMEA/Checksum.hpp"
#include "Device/Parser.hpp"
#include "NMEA/Info.hpp"
#include "Atmosphere/Temperature.hpp"


class AirmarDevice final : public AbstractDevice {
  Port &port;

  /**
   * The generic NMEA parser for this device.  It may hold internal
   * state.
   */
  NMEAParser parser;
  bool WIMDA(NMEAInputLine &line, NMEAInfo &info);


public:
  explicit AirmarDevice(Port &_port):port(_port) {}

public:
  bool ParseNMEA(const char *_line, NMEAInfo &info);
};



bool
AirmarDevice::WIMDA(NMEAInputLine &line, NMEAInfo &info)
{
  /*
    * $WIMDA,x.xx,a,x.x,a,a,a,*hh
    *
    * Field Number:
    *  1) barometric pressure in inches mercury
    *  2) (I)nches
    *  3) barometric pressure in bar
    *  4) (B)arometric
    *  5) air temperature
    *  6) (C)entigrade
    *  7-17) not available
    *  18) Checksum
    */

  double pressure;

  // to field 2
  line.Skip(2);

  if (!line.ReadChecked(pressure))
    return false;

  char ch = line.ReadOneChar();
  if (ch == 'B')
  {
    info.ProvideStaticPressure(AtmosphericPressure::HectoPascal(pressure*1000));
  }

  double temperature;
  if (!line.ReadChecked(temperature))
    return false;
  if (ch == 'C')
  {
    // temperature in centigrade
    info.temperature = CelsiusToKelvin(temperature);
    info.temperature_available = true;
  }

  return true;
}


bool
AirmarDevice::ParseNMEA(const char *_line, NMEAInfo &info)
{

  if (!VerifyNMEAChecksum(_line))
    return false;


  NMEAInputLine nmea_line(_line);
  char type[16];
  nmea_line.Read(type, 16);


  if (StringIsEqual(type, "$WIMDA"))
  {
    return WIMDA(nmea_line, info);
  }

  // pass anything else to the standard parser
  return parser.ParseLine(_line, info);

}


static Device *
AirmarCreateOnPort(const DeviceConfig &config, Port &com_port)
{
  return new AirmarDevice(com_port);
}


const DeviceRegister airmar_driver = {
  _T("Airmar"),
  _T("Airmar Weatherdata"),
  0,
  AirmarCreateOnPort,
};


