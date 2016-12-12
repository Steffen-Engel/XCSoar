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

#include "InfoBoxes/Content/Airspace.hpp"
#include "InfoBoxes/Data.hpp"
#include "Interface.hpp"
#include "Components.hpp"
#include "Engine/Airspace/AbstractAirspace.hpp"
#include "Computer/GlideComputer.hpp"
#include "Airspace/NearestAirspace.hpp"
#include "Screen/Canvas.hpp"
#include "Look/Look.hpp"
#include "UIGlobals.hpp"
#include "Formatter/UserUnits.hpp"

void
UpdateInfoBoxNearestAirspaceHorizontal(InfoBoxData &data)
{
  NearestAirspace nearest = NearestAirspace::FindHorizontal(CommonInterface::Basic(),
                                                            glide_computer->GetAirspaceWarnings(),
                                                            airspace_database);
  if (!nearest.IsDefined()) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromDistance(nearest.distance);
  data.SetComment(nearest.airspace->GetName());
}

void
UpdateInfoBoxNearestAirspaceVertical(InfoBoxData &data)
{
  NearestAirspace nearest = NearestAirspace::FindVertical(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          glide_computer->GetAirspaceWarnings(),
                                                          airspace_database);
  if (!nearest.IsDefined()) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromArrival(nearest.distance);
  data.SetComment(nearest.airspace->GetName());
}


typedef enum {ASnone, ASok, ASwarn, ASalert} ASWARNING;

static Color WarnColor(float dist, float ok, float warn, float alert)
{
  const Look &look = UIGlobals::GetLook();
  if (dist > ok)
    return (look.info_box.inverse ? COLOR_BLACK : COLOR_WHITE);
  if (dist > warn)
    return COLOR_GREEN;
  if (dist > alert)
    return COLOR_YELLOW;
  else
    return COLOR_RED;

}


static void paintAirspace(Canvas &canvas, const PixelRect &rc, float dist1, const TCHAR *text1, float dist2, const TCHAR *text2, float dist3, const TCHAR *text3)
{
// warning levels in meters
// actually hard coded, shall be user parameters in the future
#define h_ok        2500
#define h_warn      1000
#define h_alert      500

#define above_ok     800
#define above_warn   500
#define above_alert  200

#define below_ok     800
#define below_warn   300
#define below_alert  100

  const auto center = rc.GetCenter();

  unsigned w = rc.GetWidth();
  unsigned h = rc.GetHeight()+5;

  unsigned strip_h = h/3;
  unsigned l = center.x-w/2;
  unsigned r = center.x+w/2+1;

  unsigned y1 = center.y-strip_h*3/2;
  unsigned y2 = center.y-strip_h*1/2;
  unsigned y3 = center.y+strip_h*1/2;
  unsigned y4 = center.y+strip_h*3/2+1;

  unsigned text_y1 = (y1+y2)/2-(canvas.GetFontHeight()/2);
  unsigned text_y2 = (y2+y3)/2-(canvas.GetFontHeight()/2);
  unsigned text_y3 = (y3+y4)/2-(canvas.GetFontHeight()/2);

  Color color;

  // upper line for airspace above
  color = WarnColor(dist1, above_ok, above_warn, above_alert);
  canvas.DrawFilledRectangle(l,  y1, r, y2, color);
  canvas.SetTextColor((color==COLOR_BLACK) ? COLOR_WHITE : COLOR_BLACK);
  canvas.DrawText(center.x-canvas.CalcTextWidth(text1)/2, text_y1, text1);

  // middle line for airspace aside
  color = WarnColor(dist2, h_ok, h_warn, h_alert);
  canvas.DrawFilledRectangle(l,  y2, r, y3, color);
  canvas.SetTextColor((color==COLOR_BLACK) ? COLOR_WHITE : COLOR_BLACK);
  canvas.DrawText(center.x-canvas.CalcTextWidth(text2)/2, text_y2, text2);

  // middle line for airspace below
  color = WarnColor(dist3, below_ok, below_warn, below_alert);
  canvas.DrawFilledRectangle(l,  y3, r, y4, color);
  canvas.SetTextColor((color==COLOR_BLACK) ? COLOR_WHITE : COLOR_BLACK);
  canvas.DrawText(center.x-canvas.CalcTextWidth(text3)/2, text_y3, text3);
}


void
InfoBoxContentAirspaces_Distance::OnCustomPaint(Canvas &canvas, const PixelRect &rc)
{
  // distances below, aside and above the next airspace

  float dist1, dist2, dist3;
  StaticString<32> name1, name2, name3;

  name1 = name2 = name3 = _T("---");
  dist1 = dist2 = dist3 = 9e99;

  NearestAirspace nearest = NearestAirspace::FindHorizontal(CommonInterface::Basic(),
                                                            glide_computer->GetAirspaceWarnings(),
                                                            airspace_database);
  if (nearest.IsDefined()) {
    dist2 = nearest.distance;
    FormatUserDistance(dist2, name2.buffer(), true, 2);
  }

  nearest = NearestAirspace::FindAbove(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          glide_computer->GetAirspaceWarnings(),
                                                          airspace_database);
  if (nearest.IsDefined()) {
    dist1 = abs(nearest.distance);
    FormatUserAltitude(dist1, name1.buffer(), true);
  }

  nearest = NearestAirspace::FindBelow(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          glide_computer->GetAirspaceWarnings(),
                                                          airspace_database);
  if (nearest.IsDefined()) {
    dist3 = abs(nearest.distance);
    FormatUserAltitude(dist3, name3.buffer(), true);
  }

  paintAirspace(canvas, rc, dist1, name1, dist2, name2, dist3, name3);
}

void
InfoBoxContentAirspaces_Distance::Update(InfoBoxData &data)
{
  data.SetCustom();
}


void
InfoBoxContentAirspaces_Name::OnCustomPaint(Canvas &canvas, const PixelRect &rc)
{
  float dist1, dist2, dist3;
  StaticString<32> name1, name2, name3;

  name1 = name2 = name3 = _T("---");
  dist1 = dist2 = dist3 = 9e99;

  NearestAirspace nearest = NearestAirspace::FindHorizontal(CommonInterface::Basic(),
                                                            glide_computer->GetAirspaceWarnings(),
                                                            airspace_database);
  if (nearest.IsDefined()) {
    dist2 = nearest.distance;
    name2 = nearest.airspace->GetName();
  }

  nearest = NearestAirspace::FindAbove(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          glide_computer->GetAirspaceWarnings(),
                                                          airspace_database);
  if (nearest.IsDefined()) {
    dist1 = abs(nearest.distance);
    name1 = nearest.airspace->GetName();
  }

  nearest = NearestAirspace::FindBelow(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          glide_computer->GetAirspaceWarnings(),
                                                          airspace_database);
  if (nearest.IsDefined()) {
    dist3 = abs(nearest.distance);
    name3 = nearest.airspace->GetName();
  }

  paintAirspace(canvas, rc, dist1, name1, dist2, name2, dist3, name3);
}

void
InfoBoxContentAirspaces_Name::Update(InfoBoxData &data)
{
  data.SetCustom();
}


static const StaticString<32>
GetAltitudeStr(AirspaceAltitude alt)
{
  StaticString<32> alt_str;

  if (alt.IsTerrain()){
    alt_str = _T("GND");
  }
  else {
    switch (alt.reference)
    {
    case AltitudeReference::AGL:
      alt_str.Format(_T("%.0fm"), alt.altitude);
      FormatUserAltitude(alt.altitude, alt_str.buffer(), true);
      break;
    case AltitudeReference::MSL:
      alt_str.Format(_T("%.0fm"), alt.altitude);
      FormatUserAltitude(alt.altitude, alt_str.buffer(), true);
      break;
    case AltitudeReference::STD:
      alt_str.Format(_T("FL%.0f"), alt.flight_level);
      break;
    default:
      alt_str = _T("---");
    }
  }
  return alt_str;
}


void
InfoBoxContentAirspaces_Altitude::OnCustomPaint(Canvas &canvas, const PixelRect &rc)
{
  float dist1, dist2, dist3;
  StaticString<32> name1, name2, name3;

  name1 = name2 = name3 = _T("---");
  dist1 = dist2 = dist3 = 9e99;

  NearestAirspace nearest = NearestAirspace::FindHorizontal(CommonInterface::Basic(),
                                                            glide_computer->GetAirspaceWarnings(),
                                                            airspace_database);
  if (nearest.IsDefined()) {
    dist2 = nearest.distance;
    name2.Format(_T("%s - %s"),
                 GetAltitudeStr(nearest.airspace->GetBase()).c_str(),
                 GetAltitudeStr(nearest.airspace->GetTop()).c_str());
  }

  nearest = NearestAirspace::FindAbove(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          glide_computer->GetAirspaceWarnings(),
                                                          airspace_database);
  if (nearest.IsDefined()) {
    dist1 = abs(nearest.distance);
    name1.Format(_T("%s - %s"),
                 GetAltitudeStr(nearest.airspace->GetBase()).c_str(),
                 GetAltitudeStr(nearest.airspace->GetTop()).c_str());
  }

  nearest = NearestAirspace::FindBelow(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          glide_computer->GetAirspaceWarnings(),
                                                          airspace_database);
  if (nearest.IsDefined()) {
    dist3 = abs(nearest.distance);
    name3.Format(_T("%s - %s"),
                 GetAltitudeStr(nearest.airspace->GetBase()).c_str(),
                 GetAltitudeStr(nearest.airspace->GetTop()).c_str());
  }

  paintAirspace(canvas, rc, dist1, name1, dist2, name2, dist3, name3);
}

void
InfoBoxContentAirspaces_Altitude::Update(InfoBoxData &data)
{
  data.SetCustom();
}
