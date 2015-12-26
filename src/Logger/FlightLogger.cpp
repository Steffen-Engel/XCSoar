/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2015 The XCSoar Project
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

#include "FlightLogger.hpp"
#include "NMEA/MoreData.hpp"
#include "NMEA/Derived.hpp"
#include "IO/TextWriter.hpp"

// Extension stuff for summarized Flightlogs...

// needed for max/min load and max VIAS
//#define YALL
#ifdef YALL
  #include "Device/Driver/yAll/yAll.h"
#endif

// needed for nearest airport
#include "Engine/Task/Unordered/AlternateList.hpp"
#include "Components.hpp"
#include "Task/ProtectedTaskManager.hpp"

#include "IO/LineReader.hpp"
#include "IO/FileLineReader.hpp"
#include "Util/Error.hxx"
#include "IO/TextWriter.hpp"
#include "Plane/Plane.hpp"
//#include "IO/TextFile.hxx"
#include "Interface.hpp"
#include "LogFile.hpp"
//#include "windef.h"
#include "LocalPath.hpp"
//#include "OS/FileUtil.hpp"
#include "Engine/Waypoint/Waypoint.hpp"
//#include "InfoBoxes/Panel/Panel.hpp"



void
FlightLogger::Reset()
{
  last_time = fixed(0);
  seen_on_ground = seen_flying = false;
  start_time.Clear();
  landing_time.Clear();
}

void
FlightLogger::LogEvent(const BrokenDateTime &date_time, const char *type)
{
  assert(type != nullptr);

  TextWriter writer(path, true);
  if (!writer.IsOpen())
    /* Shall we log this error?  Not sure, because when this happens,
       usually the log file cannot be written either .. */
    return;

  /* XXX log pilot name, glider, airfield name */

  writer.FormatLine("%04u-%02u-%02uT%02u:%02u:%02u %s",
                    date_time.year, date_time.month, date_time.day,
                    date_time.hour, date_time.minute, date_time.second,
                    type);
}

void
FlightLogger::TickInternal(const MoreData &basic,
                           const DerivedInfo &calculated)
{
  const FlyingState &flight = calculated.flight;

  if (seen_on_ground && flight.flying) {
    /* store preliminary start time */
    start_time = basic.date_time_utc;

    if (!flight.on_ground) {
      /* start was confirmed (not on ground anymore): log it */
      seen_on_ground = false;

      LogEvent2(start_time, "start");

      start_time.Clear();
    }
  }

  if (seen_flying && flight.on_ground) {
    /* store preliminary landing time */
    landing_time = basic.date_time_utc;

    if (!flight.flying) {
      /* landing was confirmed (not on ground anymore): log it */
      seen_flying = false;

      LogEvent2(landing_time, "landing");

      landing_time.Clear();
    }
  }

  if (flight.flying && !flight.on_ground)
    seen_flying = true;

  if (!flight.flying && flight.on_ground)
    seen_on_ground = true;
}

void
FlightLogger::Tick(const MoreData &basic, const DerivedInfo &calculated)
{
  assert(!path.IsNull());

  if (basic.gps.replay || basic.gps.simulator)
    return;

  if (!basic.time_available || !basic.date_time_utc.IsDatePlausible())
    /* can't work without these */
    return;

  if (positive(last_time)) {
    auto time_delta = basic.time - last_time;
    if (negative(time_delta) || time_delta > fixed(300))
      /* reset on time warp (positive or negative) */
      Reset();
    else if (time_delta < fixed(0.5))
      /* not enough time has passed since the last call: ignore this
         GPS fix, don't update last_time, just return */
      return;
    else
      TickInternal(basic, calculated);
  }

  last_time = basic.time;
}


void
FlightLogger::LogEvent2(const BrokenDateTime &date_time, const char *type)
{
  assert(type != nullptr);
  LogDebug(_T("logging event "));

  static BrokenDateTime start_time;

  TextWriter writer(path, true);
  if (!writer.IsOpen())
  {
    /* Shall we log this error?  Not sure, because when this happens,
       usually the log file cannot be written either .. */
    return;
  }

  StaticString<64> temp;
  const FlyingState &flight = CommonInterface::Calculated().flight;

  /* XXX log pilot name, glider, airfield name */
  if (strcmp(type, "start") == 0)
  {

    GetAirfield(true);

    if (!flight.flying) {
      writer.NewLine();
      writer.Write("Logevent start with no state of flying");
    }

    // save the takeofftime for calculation of flight_time later
    start_time = date_time;
    start_time.second = 0;    // no seconds needed in logbook

    if (flight.flight_time>=fixed(0)) {
      sprintf((char*)temp.buffer(), "%02u.%02u.%04u  %02u:%02u ",
                        date_time.day, date_time.month, date_time.year,
                        date_time.hour, date_time.minute);
      writer.NewLine();
      writer.Write(temp.buffer());
    }
  }
  else if (strcmp(type, "landing") == 0)
  {
    GetAirfield(false);
    if (!flight.flying && positive(flight.flight_time)) {
      sprintf((char*)temp.buffer(), "  %02u:%02u ",
                        date_time.hour, date_time.minute);
      writer.Write(temp.buffer());
      writer.Write("  ");
      LogDebug(_T("debug 01"));
      int seconds = 0;
      if (start_time.IsPlausible())
      {
        seconds = date_time - start_time;
      }
      LogDebug(_T("debug 02"));
      sprintf((char*)temp.buffer(), " %02u:%02u",
                        seconds / 3600, (seconds / 60) % 60);
      writer.Write(temp.buffer());
      LogDebug(_T("debug 03"));

#ifdef YALL
      if (MaxValues.max_q > 100)
      {
        float speed;
        speed = sqrt(2.0*MaxValues.max_q/1.225)*3.6;
        sprintf((char*)temp.buffer(), " | %4.1fg %4.1fg %4.0fkm/h", 0.01*MaxValues.min_g, 0.01*MaxValues.max_g, speed
                          );
        writer.Write(temp.buffer());
      }
#endif

      temp.Format(_T(" |  %s  --  %s"), TakeoffInfo.c_str(), LandingInfo.c_str());
      writer.Write(temp.buffer());

      writer.Flush();

      WriteSummary(date_time);
    }
    else{
      writer.Write("Still flying?");
    }

  }
  else
  {
  writer.FormatLine("%04u-%02u-%02uT%02u:%02u:%02u %s",
                    date_time.year, date_time.month, date_time.day,
                    date_time.hour, date_time.minute, date_time.second,
                    type);
  }

}


bool
FlightLogger::GetAirfield(bool takeoff)
{

  ProtectedTaskManager::Lease lease(*protected_task_manager);
  const AlternateList &alternates = lease->GetAlternates();
  unsigned index = 0;

  const AlternatePoint *alternate;
  if (!alternates.empty()) {
    if (index >= alternates.size())
      index = alternates.size() - 1;

    alternate = &alternates[index];
  } else {
    alternate = NULL;
  }

  if (alternate == NULL) {
    return false;
  }

  if (takeoff)
  {
    LogDebug(_T("takeoff at Airport %s "),
               alternate->waypoint->name.c_str());
    TakeoffInfo = alternate->waypoint->name.c_str();
  }
  else
  {
    if (alternate->solution.vector.distance < 1500)
    {
      LogDebug(_T("landing at Airport %s "),
                 alternate->waypoint->name.c_str());
      LandingInfo = alternate->waypoint->name.c_str();
    }
    else
    {
      LandingInfo.Format(_T("Airport %s bearing %.0f° in %.1fkm"),
                         alternate->waypoint->name.c_str(),
                         alternate->solution.vector.bearing.AbsoluteDegrees(),
                         alternate->solution.vector.distance/1000);
      LogDebug(_T("%s"), LandingInfo.c_str());
    }
  }

  return true;
}

void
FlightLogger::WriteSummary(const BrokenDateTime &date_time)
{
  FileLineReader FlightLog(path, IgnoreError(), Charset::AUTO);

  const auto out_path = MakeLocalPath(_T("out"));

  const auto outfile    = AllocatedPath::Build(out_path, "Flugbuch.log");


  TextWriter writer(outfile, false);
  if (!writer.IsOpen())
  {
    LogDebug(_T("can't open %s"), outfile.c_str());
    return;
  }

  Plane plane = CommonInterface::GetComputerSettings().plane;


  StaticString<32> registration;
  StaticString<32> type;

  if (plane.registration.empty())
    registration = "D-0000";
  else
    registration = plane.registration.c_str();

  if (plane.type.empty())
    type = "type";
  else
    type = plane.type.c_str();

  struct {
    int count;
    int minutes;
  } flight;
  flight.count = 0;
  flight.minutes = 0;

  char *Line;
  while ((Line = FlightLog.ReadLine()) != NULL)
  {
                          //    09.04.2014  15:46   15:47    00:01
    int day, month, year, starthour, startminute, landhour, landminute, flighthours, flightminutes;
    int result = sscanf(Line, "%02d.%02d.%04d  %02d:%02d   %02d:%02d    %02d:%02d",
                        &day, &month, &year, &starthour, &startminute, &landhour, &landminute, &flighthours, &flightminutes);
    if (result == 9) {
      // valid line with entry of flight
      if ((day == date_time.day) && (month == date_time.month) && (year == date_time.year))
      {
        // it's a flight of today
        flight.count++;
        flight.minutes += flighthours*60+flightminutes;
#if true
        writer.WriteLine(Line);
#else
        writer.FormatLine("%s %s %02d.%02d.%04d  %02d:%02d  %02d:%02d  %02d:%02d",
                            type.c_str(), registration.c_str(), date_time.day, date_time.month, date_time.year,
                            starthour, startminute, landhour, landminute, flighthours, flightminutes);
#endif
      }
    }
  }

  if (flight.count > 0)
  {
    writer.NewLine();
    writer.FormatLine("%s %s flights on %02d.%02d.%04d: %d flights, %02d:%02d hours",
                      type.c_str(), registration.c_str(), date_time.day, date_time.month, date_time.year,
                      flight.count, flight.minutes/60, flight.minutes%60);
  }

}
