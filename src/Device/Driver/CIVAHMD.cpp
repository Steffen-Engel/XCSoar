// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project


#include "Device/Driver/CIVAHMD.hpp"
#include "Device/Driver.hpp"
#include "Device/Parser.hpp"

#include "Units/System.hpp"
#include "NMEA/Checksum.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"
#include "NMEA/MoreData.hpp"
#include "LogFile.hpp"
#include "Audio/Sound.hpp"
#include "Audio/VarioGlue.hpp"

#include "FLARM/Error.hpp"
#include "FLARM/Version.hpp"
#include "FLARM/Status.hpp"
#include "FLARM/List.hpp"
#include "Geo/Math.hpp"

using std::string_view_literals::operator""sv;

class CivaHmdDevice : public AbstractDevice {
public:
  /* virtual methods from class Device */
  bool ParseNMEA(const char *line, struct NMEAInfo &info) override;
  void OnSensorUpdate(const MoreData &basic) override;
};

// global var declaration
int CIVATargetId = 0;
int CIVAIsBeeping = 0;

GeoPoint MyLocation;

static bool
cPHMD0(NMEAInputLine &line, [[maybe_unused]] NMEAInfo &info)
{
  /*
  $PHMD0,AEAB01,5231.4761,N,01027.7460,E,1201,1

    0 Id of received logger (AEABnn)
    1 latitude
    2 side of latitude (N/S)
    3 longitude
    4 side of longitude (E/W)
    5 altitude above start (m)
    6 track
    7 speed
    8 beeper is on
    9 max positive g load since last send
   10 max negative g load since last send
  */

   // speed up the redraw after change of Id, if changed.
  static int lastFocus = 0;
  if (lastFocus != CIVATargetId)
  {
   info.Reset();
   lastFocus = CIVATargetId;
  }

  // read ID
  [[maybe_unused]] int HmdId;

  // 0 id, 6 digit hex
  char id_string[16];
  line.Read(id_string, 16);
  HmdId = (int)strtol(id_string, NULL, 16)%256;
  // 1, 2, 3, 4 position
  GeoPoint location;
  [[maybe_unused]] bool valid_location = NMEAParser::ReadGeoPoint(line, location);

  // 5 altitude
  double altitude;
  if (line.ReadChecked(altitude))
  {
  }

  // 6 track
  double track;
  if (line.ReadChecked(track))
  {
  }

  // 7 speed
  double speed;
  if (line.ReadChecked(speed))
  {
  }

  // 8 is beeping
  int beeper;
  if (line.ReadChecked(beeper))
  {
  }

  // 9 is  max pos g-Load
  double max_g_load;
  if (line.ReadChecked(max_g_load))
  {
  }
  // 10 is max neg g-Load
  double min_g_load;
  if (line.ReadChecked(min_g_load))
  {
  }

  BrokenDateTime dt = BrokenDateTime::NowUTC();

  info.date_time_utc.second = dt.second;
  info.date_time_utc.minute = dt.minute;
  info.date_time_utc.hour = dt.hour;

  info.date_time_utc.day = dt.day;
  info.date_time_utc.month = dt.month;
  info.date_time_utc.year = dt.year;


  TimeStamp this_time;
  BrokenTime broken_time = BrokenTime(info.date_time_utc.hour, info.date_time_utc.minute, (unsigned)info.date_time_utc.second);
          this_time = TimeStamp{broken_time.DurationSinceMidnight()};
  static TimeStamp last_time;

  if (NMEAParser::TimeHasAdvanced(this_time, last_time, info))
  {
  }

  info.location_available.Update(info.clock);
  info.alive.Update(info.clock);

  if (HmdId == CIVATargetId)
  {

    if (valid_location)
    {
      info.location_available.Update(info.clock);
      info.location = location;
    }

    info.ProvideBaroAltitudeTrue(altitude);
    info.track = Angle::Degrees(track);
    info.track_available.Update(info.clock);

    info.ground_speed = Units::ToSysUnit(speed, Unit::METER_PER_SECOND);
    info.ground_speed_available.Update(info.clock);

    if (max_g_load > min_g_load)
      info.acceleration.ProvideGLoad(max_g_load, true);
    else
      info.acceleration.ProvideGLoad(-1*min_g_load, true);


    CIVAIsBeeping = beeper;
    if (CIVAIsBeeping)
    {
      PlayResource(_T("IDR_WAV_BEEPCIVA"));
    }
  }
  else
  {
    info.flarm.traffic.modified.Update(info.clock);

    FlarmTraffic traffic;
    traffic.location = location;
    traffic.location_available = true;
    traffic.alarm_level = FlarmTraffic::AlarmType::NONE;
    traffic.id = FlarmId::Parse(id_string, nullptr);
    traffic.type = FlarmTraffic::AircraftType::GLIDER;


    Angle bearing;
    double distance;
    DistanceBearing(MyLocation, location,
                    &distance, &bearing);

    traffic.relative_north = bearing.cos()*distance;
    traffic.relative_east = bearing.sin()*distance;
    traffic.relative_altitude = altitude;
    traffic.track = Angle::Degrees(track);
    traffic.turn_rate = 0;
    traffic.speed = 0;

    FlarmTraffic *flarm_slot = info.flarm.traffic.FindTraffic(traffic.id);
    if (flarm_slot == nullptr) {
      flarm_slot = info.flarm.traffic.AllocateTraffic();
      if (flarm_slot == nullptr)
        // no more slots available
        return true;

      flarm_slot->Clear();
      flarm_slot->id = traffic.id;

      info.flarm.traffic.new_traffic.Update(info.clock);
    }

    // set time of fix to current time
    flarm_slot->valid.Update(info.clock);

    flarm_slot->Update(traffic);
  }

  return true;
}



bool
CivaHmdDevice::ParseNMEA(const char *String, [[maybe_unused]] NMEAInfo &info)
{
  if (!VerifyNMEAChecksum(String))
    return false;

  NMEAInputLine line(String);

  const auto type = line.ReadView();
  if (type == "$PHMD0"sv)
    return cPHMD0(line, info);
  else
    return false;
}

static Device *
GenericCreateOnPort([[maybe_unused]] const DeviceConfig &config, [[maybe_unused]] Port &com_port)
{
  return new CivaHmdDevice();
}

const struct DeviceRegister civahmd_driver = {
  _T("civa_hmd"),
  _T("CIVA HMD"),
  0,
  GenericCreateOnPort,
};

void
CivaHmdDevice::OnSensorUpdate(const MoreData &basic)
{
  MyLocation = basic.location;
}
