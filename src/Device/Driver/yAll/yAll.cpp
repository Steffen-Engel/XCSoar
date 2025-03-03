

#include "stdint.h"
#include "CmdBuffer.h"
#include "LogFile.hpp"
#include "NMEA/Info.hpp"
#include "InfoBoxes/InfoBoxManager.hpp"
#include "io/FileOutputStream.hxx"
#include "Interface.hpp"
#include "Device/Parser.hpp"
#include "Math/SelfTimingKalmanFilter1d.hpp"

#include "LocalPath.hpp"
#include "system/FileUtil.hpp"
#include "io/FileLineReader.hpp"

#include <windef.h> // for MAX_PATH

#include <tchar.h>

#include "yAll.h"


//#define USE_KALMAN

tLoggerSetData LoggerData;


tMaxValues MaxValues;

/**
  * This Kalman filter is used to smooth the pressure input.
  */
#ifdef USE_KALMAN
KalmanFilter1d kalman_filter_static0(double(0.05));
KalmanFilter1d kalman_filter_static1(double(0.05));
KalmanFilter1d kalman_filter_dynamic(double(1));
#endif

// some time helper routines
/*
 * Return Modified Julian Day given calendar year,
 * month (1-12), and day (1-31).
 * - Valid for Gregorian dates from 17-Nov-1858.
 * - Adapted from sci.astro FAQ.
 */
static long
DateToMjd (long Year, long Month, long Day)
{
    return
        367 * Year
        - 7 * (Year + (Month + 9) / 12) / 4
        - 3 * ((Year + (Month - 9) / 7) / 100 + 1) / 4
        + 275 * Month / 9
        + Day
        + 1721028
        - 2400000;
}


/*
 * Convert Modified Julian Day to calendar date.
 * - Assumes Gregorian calendar.
 * - Adapted from Fliegel/van Flandern ACM 11/#10 p 657 Oct 1968.
 */
static void
MjdToDate (long Mjd, long *Year, long *Month, long *Day)
{
    long J, C, Y, M;

    J = Mjd + 2400001 + 68569;
    C = 4 * J / 146097;
    J = J - (146097 * C + 3) / 4;
    Y = 4000 * (J + 1) / 1461001;
    J = J - 1461 * Y / 4 + 31;
    M = 80 * J / 2447;
    *Day = J - 2447 * M / 80;
    J = M / 11;
    *Month = M + 2 - (12 * J);
    *Year = 100 * (C - 49) + Y + J;
}


/*
 * Convert GPS Week and Seconds to Modified Julian Day.
 * - Ignores UTC leap seconds.
 */
static long
GpsToMjd (long GpsCycle, long GpsWeek, long GpsSeconds)
{
    long GpsDays;

    GpsDays = ((GpsCycle * 1024) + GpsWeek) * 7 + (GpsSeconds / 86400);
    return DateToMjd(1980, 1, 6) + GpsDays;
}




cyAll::cyAll()
{
  c_state = IDLE;
  err_rcvd = false;
  dataSize = 0;

  flying = false;
  writer = nullptr;
  last_time = TimeStamp::Undefined();

  LoggerData.eta[0] = 0;
  LoggerData.eta[1] = 0;
  LoggerData.eta[2] = 0;
  LoggerData.eta[3] = 0;

  MaxValues.max_q = 0;
  MaxValues.max_g = 100;
  MaxValues.min_g = 100;
}


void cyAll::Open(Port &_port)
{
  port = &_port;

  /*
  TCHAR szFile[50];
  LocalPath(szFile, _T("Deviation.par"));
  FileLineReaderA reader(szFile);
  if (reader.error())
     return;

   LogFormat(_T("Loading Deviation from %s"), szFile);
*/
}


void cyAll::sendHeader()
{
  port->Write("$M<");
  checksum = 0;
}


void cyAll::sendByte(char value)
{
  port->Write(value);
  checksum ^= value;
}


void cyAll::sendWord(uint16_t value)
{
  sendByte(value & 0xff);
  sendByte(value >> 8);
}


void cyAll::sendLong(uint32_t value)
{
  sendWord(value & 0xffff);
  sendWord(value >> 16);
}


void cyAll::sendEnd()
{
  port->Write(checksum);
}


void cyAll::sendRequest(uint8_t value)
{
  sendHeader();
  sendByte(0);				// Payload-Size = 0
  sendByte(value);
  sendEnd();
}



void cyAll::sendRequest(uint8_t value, uint8_t parm)
{
  sendHeader();
  sendByte(1);
  sendByte(value);
  sendByte(parm);
  sendEnd();
}


void cyAll::sendRequest(uint8_t value, uint16_t parm)
{
  sendHeader();
  sendByte(2);
  sendByte(value);
  sendWord(parm);
  sendEnd();
}


void cyAll::sendRequest(uint8_t value, uint32_t parm)
{
  sendHeader();
  sendByte(4);
  sendByte(value);
  sendLong(parm);
  sendEnd();
}


void cyAll::StartLog(void)
{
  if (writer != nullptr)
    return;


  MaxValues.max_q = LoggerData.pressure[2];
  MaxValues.max_g = LoggerData.accel[2];
  MaxValues.min_g = LoggerData.accel[2];

  BrokenDateTime dt = BrokenDateTime::NowUTC();
  assert(dt.IsPlausible());

  StaticString<64> name;
  name.Format(_T("%04u-%02u-%02u_%02u-%02u.dat"), dt.year, dt.month, dt.day, dt.hour, dt.minute);

  const auto logs_path = MakeLocalPath(_T("logs"));

  const auto path = AllocatedPath::Build(logs_path, name);

  LogFormat(_T("Logging flight to %s"), path.c_str());

  file = new FileOutputStream(path, FileOutputStream::Mode::APPEND_OR_CREATE);
  writer = new BufferedOutputStream(*file);

  writer->Write("date, time, time/msec, nx/g, ny/g, nz/g, phi, psi, theta, rot_x/°/sec, rot_y/°/sec, rot_z/°/s, p_cabin/Pa, p_stat/Pa, p_diff/Pa,eta_hr, eta_qr, eta_sr, eta_fl, GPS_FIX, numSat, GPS_altitude, GPS_speed, longitude, latitude, GPS_track");
  writer->Flush();

}

void cyAll::EndLog()
{
  LogDebug(_T("Flightlogging stopped"));
  writer->Flush();

  delete writer;
  writer = nullptr;
  delete file;
  file = nullptr;
}

#if 0
gcc_pure
static inline
double ComputeNoncompVario(const double pressure, const double d_pressure)
{
  static constexpr double FACTOR(-2260.389548275485);
  static constexpr double EXPONENT(-0.8097374740609689);
  return double(FACTOR * pow(pressure, EXPONENT) * d_pressure);
}
#endif


void cyAll::evaluateCommand(uint8_t cmd, [[maybe_unused]] int dataSize, struct NMEAInfo &info)
{

  int icmd = (int)(cmd&0xFF);
  uint8_t *data;
  uint8_t datasize;

  bool FlightLogger_flying = FlightState;

  switch(icmd)
  {
    case MSP_IDENT:
      data = (uint8_t*)(&MSPData.Ident);
      datasize = sizeof(MSPData.Ident);
      for (uint8_t count = 0; count < datasize; count++)
      {
         data[count] = CmdBuffer.read8();
      }
      Ident = MSPData.Ident;
      break;
    case MSP_STATUS:
      data = (uint8_t*)(&MSPData.Status);
      datasize = sizeof(MSPData.Status);
      for (uint8_t count = 0; count < datasize; count++)
      {
         data[count] = CmdBuffer.read8();
       }
      break;
    case MSP_LOGGER_SET:
      uint8_t *data;
      data = (uint8_t*)(&LoggerData);
      for (uint8_t count = 0; count < sizeof(LoggerData); count++)
      {
        data[count] = CmdBuffer.read8();
      }

      for (int8_t count = 0; count < 4; count++)
        LoggerData.eta[count] = (LoggerData.eta[count]-1500)/5;

#if 0
      static int16_t dummy = 0;
      static int8_t increment = 1;
      if ((dummy>100) || (dummy < -100))
      {
        increment *= -1;
      }
      dummy += increment;
      for (int8_t count = 0; count < 4; count++)
        LoggerData.eta[count] = dummy;
#endif

// refine pressure value for differential pressure, offset 8 Pa
//      LoggerData.pressure[2] -= 8;
//      if (LoggerData.pressure[2] < 0)
//        LoggerData.pressure[2] = 0;


#if 0
      static uint32_t lastlog;
      if ((LoggerData.millis > lastlog+1000) || (lastlog > LoggerData.millis))
      {
        lastlog = LoggerData.millis;
        LogFormat(_T("msecs %6ld angle %4d %4d %4d alt %3d %3d %3d accel %4d %4d %4d"),
               (long int)LoggerData.millis,
               LoggerData.angle[0], LoggerData.angle[1], LoggerData.angle[2],
               LoggerData.pressure[0], LoggerData.pressure[1], LoggerData.pressure[2],
               LoggerData.accel[0], LoggerData.accel[1], LoggerData.accel[2]);
      }
#endif
#if 0
      LogDebug(_T("msecs %6ld angle %4d %4d %4d alt %3d %3d %3d accel %4d %4d %4d"),
               (long int)LoggerData.millis,
               LoggerData.angle[0], LoggerData.angle[1], LoggerData.angle[2],
               LoggerData.pressure[0], LoggerData.pressure[1], LoggerData.pressure[2],
               LoggerData.accel[0], LoggerData.accel[1], LoggerData.accel[2]);
#endif
#if 0

      LogDebug(_T("GPS fix %d numsat %d lat %d long %d, alt %d, speed %d, week %d, time %d"),
               LoggerData.GPS_FIX,
               LoggerData.GPS_numSat,
               LoggerData.GPS_latitude,
               LoggerData.GPS_longitude,
               LoggerData.GPS_altitude,
               LoggerData.GPS_speed,
               LoggerData.GPS_week,
               LoggerData.GPS_time
                              );
#endif

      info.alive.Update(info.clock);

      info.attitude.bank_angle_available.Update(info.clock);
      info.attitude.bank_angle = Angle::Degrees(0.1*LoggerData.angle[0]);

      info.attitude.pitch_angle_available.Update(info.clock);
      info.attitude.pitch_angle = Angle::Degrees(-0.1*LoggerData.angle[1]);

#if 0
      info.attitude.heading_available.Update(info.clock);
      info.attitude.heading = Angle::Degrees(0.1*LoggerData.angle[2]);
#endif

      info.acceleration.ProvideGLoad(0.01*LoggerData.accel[2], true);

#ifdef USE_KALMAN

        static double laststatictime;

        if (double(LoggerData.millis)/1000>laststatictime)
        {
          kalman_filter_static0.Update(LoggerData.pressure[0], double(0.05), double(LoggerData.millis)/1000-laststatictime);
          kalman_filter_static1.Update(LoggerData.pressure[1], double(0.05), double(LoggerData.millis)/1000-laststatictime);
        }
        laststatictime = double(LoggerData.millis)/1000;
//        info.ProvideNoncompVario(ComputeNoncompVario(kalman_filter_static.GetXAbs(),
//                                                      kalman_filter_static.GetXVel()));
        info.ProvideStaticPressure(AtmosphericPressure::Pascal(kalman_filter_static1.GetXAbs()));

        static double lastdynamictime;

        if (double(LoggerData.millis)/1000>lastdynamictime)
        {
          kalman_filter_dynamic.Update(LoggerData.pressure[2], double(0.05), double(LoggerData.millis)/1000-lastdynamictime);
        }
        lastdynamictime = double(LoggerData.millis)/1000;
        double pressure_val;
        pressure_val = kalman_filter_dynamic.GetXAbs();
        if (pressure_val < 1) pressure_val = 0;
        info.ProvideDynamicPressure(AtmosphericPressure::Pascal(pressure_val));
        if (pressure_val > MaxValues.max_q)
        {
          MaxValues.max_q = pressure_val;
        }
#else

      // deliver pressure values for altitude and speed
      if (LoggerData.pressure[1] >= 0)
      {
        info.ProvideStaticPressure(AtmosphericPressure::Pascal(LoggerData.pressure[1]));
      }
      else
      {
        info.static_pressure_available.Clear();
      }
      if (LoggerData.pressure[2] >= 0)
      {
        info.ProvideDynamicPressure(AtmosphericPressure::Pascal(LoggerData.pressure[2]));
      }
      else
      {
        info.ProvideDynamicPressure(AtmosphericPressure::Pascal(double(0)));
      }
      if (LoggerData.pressure[2] > MaxValues.max_q)
      {
        MaxValues.max_q = LoggerData.pressure[2];
      }

#endif // USE_KALMAN

      InfoBoxManager::SetDirty();

      if (FlightLogger_flying != flying)
      {
        // changed to flying?
        if (FlightLogger_flying)
        {
          StartLog();
        }
        else
        {
          EndLog();
        }

        // accept state of flying, so logging will start or stop
        flying = FlightLogger_flying;
      }

      if (LoggerData.accel[2] > MaxValues.max_g)
      {
        MaxValues.max_g = LoggerData.accel[2];
      }
      if (LoggerData.accel[2] < MaxValues.min_g)
      {
        MaxValues.min_g = LoggerData.accel[2];
      }

      if (LoggerData.GPS_FIX)
      {
        TimeStamp this_time;

        info.date_time_utc.second = (LoggerData.GPS_time/1000) % 60;
        info.date_time_utc.minute = (LoggerData.GPS_time/1000)/60 % 60;
        info.date_time_utc.hour = (LoggerData.GPS_time/1000)/60/60 % 24;

        BrokenTime broken_time = BrokenTime(info.date_time_utc.hour, info.date_time_utc.minute, (unsigned)info.date_time_utc.second);
        this_time = TimeStamp{broken_time.DurationSinceMidnight()};

        long year, month, day;
        MjdToDate (GpsToMjd(0, LoggerData.GPS_week, LoggerData.GPS_time/1000), &year, &month, &day);
        info.date_time_utc.day = day;
        info.date_time_utc.month = month;
        info.date_time_utc.year = year;

        if (NMEAParser::TimeHasAdvanced(this_time, last_time, info))
        {
          GeoPoint location;
          location.latitude = Angle::Degrees(double(LoggerData.GPS_latitude)/10000000.0);
          location.longitude = Angle::Degrees(double(LoggerData.GPS_longitude)/10000000.0);
          info.location_available.Update(info.clock);
          info.location = location;

          if (info.MovementDetected())
          {
            info.track = Angle::Degrees(double(LoggerData.GPS_heading)/10.0);
            info.track_available.Update(info.clock);
          }

          info.ground_speed = double(LoggerData.GPS_speed)/100.0;
          info.ground_speed_available.Update(info.clock);

          info.gps_altitude_available.Update(info.clock);
          info.gps_altitude = double(LoggerData.GPS_altitude);

          info.gps.real = true;

#ifdef ANDROID
          info.gps.nonexpiring_internal_gps = false;
#endif
        }

      }

      if (flying)
      {
        StaticString<256> temp;
        temp.Format(_T("%02d.%02d.%04d,%02d:%02d:%02d.%d,%8d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%6d,%6d,%5d,%4d,%4d,%4d,%4d,%1d,%2d,%5d,%6d,%9d,%9d,%4d\n"),
                            info.date_time_utc.day,
                            info.date_time_utc.month,
                            info.date_time_utc.year,
                            (LoggerData.GPS_time/1000)/60/60 % 24,
                            (LoggerData.GPS_time/1000)/60 % 60,
                            (LoggerData.GPS_time/1000) % 60,
                            (LoggerData.GPS_time/100) % 10,
                            LoggerData.millis,
                            LoggerData.accel[0], LoggerData.accel[1], LoggerData.accel[2],
                            LoggerData.angle[0], LoggerData.angle[1], LoggerData.angle[2],
                            LoggerData.rot[0], LoggerData.rot[1], LoggerData.rot[2],
                            LoggerData.pressure[0], LoggerData.pressure[1], LoggerData.pressure[2],
                            LoggerData.eta[ETA_HR], LoggerData.eta[ETA_QR], LoggerData.eta[ETA_SR], LoggerData.eta[ETA_FLAP],
                            LoggerData.GPS_FIX, LoggerData.GPS_numSat, LoggerData.GPS_altitude, LoggerData.GPS_speed,
                            LoggerData.GPS_longitude, LoggerData.GPS_latitude, LoggerData.GPS_heading
                            );
        writer->Write(temp.buffer());
      }



      break;
    default:
        //println("Don't know how to handle reply "+icmd);
      break;
  }
}



void cyAll::parse(uint8_t c, struct NMEAInfo &info)
{
static uint8_t cmd;


  if (c_state == IDLE)
  {
    c_state = (c=='$') ? HEADER_START : IDLE;
  }
  else if (c_state == HEADER_START)
  {
    c_state = (c=='M') ? HEADER_M : IDLE;
  }
  else if (c_state == HEADER_M)
  {
    if (c == '>')
    {
      c_state = HEADER_ARROW;
    }
    else if (c == '!')
    {
      c_state = HEADER_ERR;
    }
    else
    {
      c_state = IDLE;
    }
  }
  else if (c_state == HEADER_ARROW || c_state == HEADER_ERR)
  {
    /* is this an error message? */
    err_rcvd = (c_state == HEADER_ERR);        /* now we are expecting the payload size */
    dataSize = (c&0xFF);
    /* reset index variables */
    CmdBuffer.Reset();
    checksum = 0;
    checksum ^= (c&0xFF);
    /* the command is to follow */
    c_state = HEADER_SIZE;
  }
  else if (c_state == HEADER_SIZE)
  {
    cmd = (uint8_t)(c&0xFF);
    checksum ^= (c&0xFF);
    c_state = HEADER_CMD;
  }
  else if (c_state == HEADER_CMD && CmdBuffer.fill() < dataSize)
  {
    checksum ^= (c&0xFF);
    CmdBuffer.write8(c&0xFF);
  }
  else if (c_state == HEADER_CMD && CmdBuffer.fill() >= dataSize)
  {
    /* compare calculated and transferred checksum */
    if ((checksum&0xFF) == (c&0xFF)) {
      if (err_rcvd)
      {
        //System.err.println("Copter did not understand request type "+c);
      }
      else
      {
        /* we got a valid response packet, evaluate it */
        evaluateCommand(cmd, (int)dataSize, info);
      }
    } // if ((checksum&0xFF) == (c&0xFF))
    else
    {
//			printf("invalid checksum for command \n");
    }
    c_state = IDLE;
  }
}

