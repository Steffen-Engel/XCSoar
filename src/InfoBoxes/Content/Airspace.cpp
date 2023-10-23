// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "InfoBoxes/Content/Airspace.hpp"
#include "InfoBoxes/Data.hpp"
#include "Interface.hpp"
#include "Components.hpp"
#include "BackendComponents.hpp"
#include "DataComponents.hpp"
#include "Engine/Airspace/AbstractAirspace.hpp"
#include "Airspace/NearestAirspace.hpp"
#include "ui/canvas/Canvas.hpp"
#include "Look/Look.hpp"
#include "UIGlobals.hpp"
#include "Formatter/UserUnits.hpp"
#include "LogFile.hpp"

void
UpdateInfoBoxNearestAirspaceHorizontal(InfoBoxData &data) noexcept
{
  NearestAirspace nearest = NearestAirspace::FindHorizontal(CommonInterface::Basic(),
                                                            backend_components->GetAirspaceWarnings(),
                                                            *data_components->airspaces);
  if (!nearest.IsDefined()) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromDistance(nearest.distance);
  data.SetComment(nearest.airspace->GetName());
}

void
UpdateInfoBoxNearestAirspaceVertical(InfoBoxData &data) noexcept
{
  NearestAirspace nearest = NearestAirspace::FindVertical(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          backend_components->GetAirspaceWarnings(),
                                                          *data_components->airspaces);
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

  int w = rc.GetWidth();
  int h = rc.GetHeight()+5;

  int strip_h = h/3;
  int l = center.x-w/2;
  int r = center.x+w/2+1;

  int y1 = center.y-strip_h*3/2;
  int y2 = center.y-strip_h*1/2;
  int y3 = center.y+strip_h*1/2;
  int y4 = center.y+strip_h*3/2+1;

  int text_y1 = (y1+y2)/2-(canvas.GetFontHeight()/2);
  int text_y2 = (y2+y3)/2-(canvas.GetFontHeight()/2);
  int text_y3 = (y3+y4)/2-(canvas.GetFontHeight()/2);

  const Look &look = UIGlobals::GetLook();
  canvas.Select(look.info_box.unit_font);
  Color color;


  // upper line for airspace above
  color = WarnColor(dist1, above_ok, above_warn, above_alert);
  canvas.DrawFilledRectangle({{l,  y1}, PixelSize{r, strip_h}}, color);
  canvas.SetTextColor((color==COLOR_BLACK) ? COLOR_WHITE : COLOR_BLACK);
  if (color == COLOR_RED)
    canvas.SetTextColor(COLOR_WHITE);
  canvas.DrawText({(int)center.x-(int)canvas.CalcTextWidth(text1)/2, text_y1}, text1);

  // middle line for airspace aside
  color = WarnColor(dist2, h_ok, h_warn, h_alert);
  canvas.DrawFilledRectangle({{l,  y2}, PixelSize{r, strip_h}}, color);
  canvas.SetTextColor((color==COLOR_BLACK) ? COLOR_WHITE : COLOR_BLACK);
  if (color == COLOR_RED)
    canvas.SetTextColor(COLOR_WHITE);
  canvas.DrawText({(int)center.x-(int)canvas.CalcTextWidth(text2)/2, text_y2}, text2);

  // middle line for airspace below
  color = WarnColor(dist3, below_ok, below_warn, below_alert);
  canvas.DrawFilledRectangle({{l,  y3}, PixelSize{r, strip_h}}, color);
  canvas.SetTextColor((color==COLOR_BLACK) ? COLOR_WHITE : COLOR_BLACK);
  if (color == COLOR_RED)
    canvas.SetTextColor(COLOR_WHITE);
  canvas.DrawText({(int)center.x-(int)canvas.CalcTextWidth(text3)/2, text_y3}, text3);
h= l*r;
}


void
InfoBoxContentAirspaces_Distance::OnCustomPaint(Canvas &canvas, const PixelRect &rc) noexcept
{
  // distances below, aside and above the next airspace

  float dist1, dist2, dist3;
  StaticString<32> name1, name2, name3;

  name1 = name2 = name3 = _T("---");
  dist1 = dist2 = dist3 = 9e99;

  NearestAirspace nearest = NearestAirspace::FindHorizontal(CommonInterface::Basic(),
                                                            backend_components->GetAirspaceWarnings(),
                                                            *data_components->airspaces);
  if (nearest.IsDefined()) {
    dist2 = nearest.distance;
    FormatUserDistance(dist2, name2.buffer(), true, 2);
  }

  nearest = NearestAirspace::FindAbove(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          *backend_components->GetAirspaceWarnings(),
                                                          *data_components->airspaces);
  if (nearest.IsDefined()) {
    dist1 = abs(nearest.distance);
    FormatUserAltitude(dist1, name1.buffer(), true);
  }

  nearest = NearestAirspace::FindBelow(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          *backend_components->GetAirspaceWarnings(),
                                                          *data_components->airspaces);
  if (nearest.IsDefined()) {
    dist3 = abs(nearest.distance);
    FormatUserAltitude(dist3, name3.buffer(), true);
  }

  paintAirspace(canvas, rc, dist1, name1, dist2, name2, dist3, name3);
}

void
InfoBoxContentAirspaces_Distance::Update(InfoBoxData &data) noexcept
{
static int count;
  data.SetCustom(count++);
}


void
InfoBoxContentAirspaces_Name::OnCustomPaint(Canvas &canvas, const PixelRect &rc) noexcept
{
  float dist1, dist2, dist3;
  StaticString<32> name1, name2, name3;

  name1 = name2 = name3 = _T("---");
  dist1 = dist2 = dist3 = 9e99;

  NearestAirspace nearest = NearestAirspace::FindHorizontal(CommonInterface::Basic(),
                                                            backend_components->GetAirspaceWarnings(),
                                                            *data_components->airspaces);
  if (nearest.IsDefined()) {
    dist2 = nearest.distance;
    name2 = nearest.airspace->GetName();
    if (name2.length()>11) {
      name2.Truncate(11);
    }
  }

  nearest = NearestAirspace::FindAbove(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          *backend_components->GetAirspaceWarnings(),
                                                          *data_components->airspaces);
  if (nearest.IsDefined()) {
    dist1 = abs(nearest.distance);
    name1 = nearest.airspace->GetName();
    if (name1.length()>11) {
      name1.Truncate(11);
    }
  }

  nearest = NearestAirspace::FindBelow(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          *backend_components->GetAirspaceWarnings(),
                                                          *data_components->airspaces);
  if (nearest.IsDefined()) {
    dist3 = abs(nearest.distance);
    name3 = nearest.airspace->GetName();
    if (name3.length()>11) {
      name3.Truncate(11);
    }
  }

  paintAirspace(canvas, rc, dist1, name1, dist2, name2, dist3, name3);
}

void
InfoBoxContentAirspaces_Name::Update(InfoBoxData &data) noexcept
{
static int count;
  data.SetCustom(count++);
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
InfoBoxContentAirspaces_Altitude::OnCustomPaint(Canvas &canvas, const PixelRect &rc) noexcept
{
  float dist1, dist2, dist3;
  StaticString<32> name1, name2, name3;

  name1 = name2 = name3 = _T("---");
  dist1 = dist2 = dist3 = 9e99;

  NearestAirspace nearest = NearestAirspace::FindHorizontal(CommonInterface::Basic(),
                                                            backend_components->GetAirspaceWarnings(),
                                                            *data_components->airspaces);
  if (nearest.IsDefined()) {
    dist2 = nearest.distance;
    name2.Format(_T("%s-%s"),
                 GetAltitudeStr(nearest.airspace->GetBase()).c_str(),
                 GetAltitudeStr(nearest.airspace->GetTop()).c_str());
  }

  nearest = NearestAirspace::FindAbove(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          *backend_components->GetAirspaceWarnings(),
                                                          *data_components->airspaces);
  if (nearest.IsDefined()) {
    dist1 = abs(nearest.distance);
    name1.Format(_T("%s-%s"),
                 GetAltitudeStr(nearest.airspace->GetBase()).c_str(),
                 GetAltitudeStr(nearest.airspace->GetTop()).c_str());
  }

  nearest = NearestAirspace::FindBelow(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          *backend_components->GetAirspaceWarnings(),
                                                          *data_components->airspaces);
  if (nearest.IsDefined()) {
    dist3 = abs(nearest.distance);
    name3.Format(_T("%s-%s"),
                 GetAltitudeStr(nearest.airspace->GetBase()).c_str(),
                 GetAltitudeStr(nearest.airspace->GetTop()).c_str());
  }

  paintAirspace(canvas, rc, dist1, name1, dist2, name2, dist3, name3);
}

void
InfoBoxContentAirspaces_Altitude::Update(InfoBoxData &data) noexcept
{
static int count;
  data.SetCustom(count++);
}
