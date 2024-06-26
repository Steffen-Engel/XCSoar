
// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "FlightLogger.hpp"
#include "NMEA/MoreData.hpp"
#include "NMEA/Derived.hpp"
#include "io/FileOutputStream.hxx"
#include "io/BufferedOutputStream.hxx"
#include "LogFile.hpp"
#include "system/FileUtil.hpp"


// Extension stuff for summarized Flightlogs...
#include "Engine/Task/TaskManager.hpp"

#ifdef ANDROID
#include "Android/Environment.hpp"
#include "Android/Context.hpp"
#endif

// needed for max/min load and max VIAS
#if __has_include("Device/Driver/yAll/yAll.h")
  #define YALL
  #include "Device/Driver/yAll/yAll.h"
#endif

// needed for nearest airport
#include "Engine/Task/Unordered/AlternateList.hpp"
#include "Components.hpp"
#include "Task/ProtectedTaskManager.hpp"

#include "io/LineReader.hpp"
#include "io/FileLineReader.hpp"
//#include "Util/Error.hpp"
#include "io/FileOutputStream.hxx"
#include "Plane/Plane.hpp"
#include "Interface.hpp"
#include "LogFile.hpp"
#include "LocalPath.hpp"
#include "Engine/Waypoint/Waypoint.hpp"
#include "BackendComponents.hpp"




void
FlightLogger::Reset()
{
#ifdef ANDROID
  const auto env = Java::GetEnv();
  auto primary_path = Environment::GetExternalStoragePublicDirectory(env, "Documents/XCSoar");
#else
  auto primary_path = GetPrimaryDataPath();
#endif
  Directory::Create(primary_path);
  auto out_path = AllocatedPath::Build(primary_path, _T("out"));
  Directory::Create(out_path);
  auto old_path = AllocatedPath::Build(out_path, _T("old"));
  Directory::Create(old_path);

  last_time = TimeStamp::Undefined();
  seen_flying = false;
  seen_on_ground = true;
  start_time.Clear();
  landing_time.Clear();
}

void
FlightLogger::LogEvent(const BrokenDateTime &date_time, const char *type)
try {
  assert(type != nullptr);

  FileOutputStream file(path, FileOutputStream::Mode::APPEND_OR_CREATE);
  BufferedOutputStream writer(file);

  /* XXX log pilot name, glider, airfield name */

  writer.Fmt("{:04}-{:02}-{:02}T{:02}:{:02}:{:02} {}\n",
             date_time.year, date_time.month, date_time.day,
             date_time.hour, date_time.minute, date_time.second,
             type);

  writer.Flush();
  file.Sync();
  file.Commit();
} catch (...) {
  LogError(std::current_exception());
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

      LogEvent(start_time, "start");
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

      LogEvent(landing_time, "landing");
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
  assert(path != nullptr);

  if (basic.gps.replay || basic.gps.simulator)
    return;

  if (!basic.time_available || !basic.date_time_utc.IsDatePlausible())
    /* can't work without these */
    return;

  if (last_time.IsDefined()) {
    auto time_delta = basic.time - last_time;
    if (time_delta.count() < 0 || time_delta > std::chrono::minutes{5})
      /* reset on time warp (positive or negative) */
      Reset();
    else if (time_delta < std::chrono::milliseconds{500})
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
try {
  assert(type != nullptr);
  LogFormat(_T("logging event %s"), type);

  static BrokenDateTime start_time;

#ifdef ANDROID
  const auto env = Java::GetEnv();
  auto out_path = Environment::GetExternalStoragePublicDirectory(env, "Documents/XCSoar");
#else
  auto out_path = GetPrimaryDataPath();
#endif
  Directory::Create(out_path);

  // open logbook for parsing flights
  const auto logbookpath = AllocatedPath::Build(out_path, _T("flightlog.txt"));

  FileOutputStream file(logbookpath, FileOutputStream::Mode::APPEND_OR_CREATE);
  BufferedOutputStream writer(file);

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

    if (flight.flight_time.count()>=0.0) {
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
    if (!flight.flying && (flight.flight_time.count() > 0.0)) {
      sprintf((char*)temp.buffer(), "  %02u:%02u ",
                        date_time.hour, date_time.minute);
      writer.Write(temp.buffer());
      writer.Write("  ");
      std::chrono::system_clock::duration seconds;
      if (start_time.IsPlausible())
      {
        seconds = date_time - start_time;
      }
      auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(seconds);
      sprintf((char*)temp.buffer(), " %02u:%02u",
                        (unsigned int)f_secs.count() / 3600, ((unsigned int)f_secs.count() / 60) % 60);
      writer.Write(temp.buffer());

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
    temp.Format(_T("%04u-%02u-%02uT%02u:%02u:%02u %s"),
                    date_time.year, date_time.month, date_time.day,
                    date_time.hour, date_time.minute, date_time.second,
                    type);
    writer.Write(temp.buffer());
  }

  writer.Flush();
  file.Sync();
  file.Commit();
} catch (...) {
  LogError(std::current_exception());
}

bool
FlightLogger::GetAirfield(bool takeoff)
{
  ProtectedTaskManager::Lease lease(*backend_components->protected_task_manager);
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
    LogFormat(_T("takeoff at Airport %s "),
               alternate->waypoint->name.c_str());
    TakeoffInfo = alternate->waypoint->name.c_str();
  }
  else
  {
    if (alternate->solution.vector.distance < 1500)
    {
      LogFormat(_T("landing at Airport %s "),
                 alternate->waypoint->name.c_str());
      LandingInfo = alternate->waypoint->name.c_str();
    }
    else
    {
      LandingInfo.Format(_T("Airport %s bearing %.0f in %.1fkm"),
                         alternate->waypoint->name.c_str(),
                         alternate->solution.vector.bearing.AbsoluteDegrees(),
                         alternate->solution.vector.distance/1000);
      LogFormat(_T("%s"), LandingInfo.c_str());
    }
  }

  return true;
}

void
FlightLogger::WriteSummary(const BrokenDateTime &date_time)
try{
#ifdef ANDROID
  const auto env = Java::GetEnv();
  auto primary_path = Environment::GetExternalStoragePublicDirectory(env, "Documents/XCSoar");
#else
  auto primary_path = GetPrimaryDataPath();
#endif

  // open logbook for parsing flights
  const auto logbookpath = AllocatedPath::Build(primary_path, _T("flightlog.txt"));
  FileLineReaderA FlightLog(logbookpath);

  const auto out_path = AllocatedPath::Build(primary_path, _T("out"));
  Directory::Create(out_path);

  StaticString<64> temp;
  sprintf((char*)temp.buffer(), "%04u%02u%02u%02u%02u.txt",
                    date_time.year, date_time.month, date_time.day,
                    date_time.hour, date_time.minute);

  const auto outfile    = AllocatedPath::Build(out_path, temp);


  FileOutputStream file(outfile, FileOutputStream::Mode::CREATE);
  BufferedOutputStream writer(file);

  Plane plane = CommonInterface::GetComputerSettings().plane;


  StaticString<32> registration;
  StaticString<32> type;

  if (plane.registration.empty())
    registration = _T("D-0000");
  else
    registration = plane.registration.c_str();

  if (plane.type.empty())
    type = _T("type");
  else
    type = plane.type.c_str();

  struct {
    int count;
    int minutes;
  } flight;
  flight.count = 0;
  flight.minutes = 0;

  TCHAR *Line;
  while ((Line = (TCHAR *)FlightLog.ReadLine()) != NULL)
  {
                          //    09.04.2014  15:46   15:47    00:01
    int day, month, year, starthour, startminute, landhour, landminute, flighthours, flightminutes;
#ifdef WIN32
    int result = swscanf(Line, _T("%02d.%02d.%04d  %02d:%02d   %02d:%02d    %02d:%02d"),
                        &day, &month, &year, &starthour, &startminute, &landhour, &landminute, &flighthours, &flightminutes);
#else
    int result = sscanf(Line, _T("%02d.%02d.%04d  %02d:%02d   %02d:%02d    %02d:%02d"),
                        &day, &month, &year, &starthour, &startminute, &landhour, &landminute, &flighthours, &flightminutes);
#endif
    if (result == 9) {
      // valid line with entry of flight
      if ((day == date_time.day) && (month == date_time.month) && (year == date_time.year))
      {
        // it's a flight of today
        flight.count++;
        flight.minutes += flighthours*60+flightminutes;
#if true
        writer.Write(Line);
        writer.NewLine();
#else
        StaticString<128> temp;
        temp.Format(_T("%s %s %02d.%02d.%04d  %02d:%02d  %02d:%02d  %02d:%02d"),
                            type.c_str(), registration.c_str(), date_time.day, date_time.month, date_time.year,
                            starthour, startminute, landhour, landminute, flighthours, flightminutes);
        writer.Write(temp.buffer());
#endif
      }
    }
  }

  if (flight.count > 0)
  {
    writer.NewLine();
    StaticString<128> temp;
    temp.Format(_T("%s %s flights on %02d.%02d.%04d: %d flights, %02d:%02d hours"),
                      type.c_str(), registration.c_str(), date_time.day, date_time.month, date_time.year,
                      flight.count, flight.minutes/60, flight.minutes%60);
    writer.Write(temp.buffer());
  }

  writer.Flush();
  file.Sync();
  file.Commit();

} catch (...) {
  LogError(std::current_exception());
}
