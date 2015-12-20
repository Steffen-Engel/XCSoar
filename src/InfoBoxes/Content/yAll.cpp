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

#include "InfoBoxes/Content/yAll.hpp"
#include "InfoBoxes/Data.hpp"
#include "Interface.hpp"
#include "Renderer/HorizonRenderer.hpp"
#include "Hardware/Battery.hpp"
#include "OS/SystemLoad.hpp"
#include "Language/Language.hpp"
#include "UIGlobals.hpp"
#include "Look/Look.hpp"
#include "Device/Driver/yAll/yAll.h"

#include "Screen/Layout.hpp"
#include "Util/Clamp.hpp"
#include "Screen/Canvas.hpp"


#ifdef HAVE_MEM_INFO
#include "Formatter/ByteSizeFormatter.hpp"
#endif

#include <tchar.h>


void
InfoBoxContentControls::OnCustomPaint(Canvas &canvas, const PixelRect &rc)
{
    Brush marker_brush, stick_brush;
    Pen frame_pen, stick_pen;


    const RasterPoint center = rc.GetCenter();

    const int radius = std::min(rc.right - rc.left, rc.bottom - rc.top) / 2
      - Layout::Scale(2);

    // draw frame for control defelections
    frame_pen.Create(Layout::Scale(1), COLOR_GRAY);
    canvas.Select(frame_pen);
    canvas.DrawLine(center.x - radius, center.y + radius,
                    center.x + radius, center.y + radius);
    canvas.DrawLine(center.x - radius, center.y - radius,
                    center.x + radius, center.y - radius);
    canvas.DrawLine(center.x - radius, center.y + radius,
                    center.x - radius, center.y - radius);
    canvas.DrawLine(center.x + radius, center.y + radius,
                    center.x + radius, center.y - radius);

    stick_brush.Create(COLOR_ORANGE);
    stick_pen.Create(1, COLOR_BLACK);
    canvas.Select(stick_pen);
    canvas.Select(stick_brush);
    // paint stick
    canvas.DrawSegment(center.x + LoggerData.eta[ETA_QR] * radius / 100, center.y + LoggerData.eta[ETA_HR] * radius / 100, Layout::Scale(3), Angle::Degrees(0), Angle::Degrees(360), true);

    marker_brush.Create(COLOR_GRAY);
    canvas.Select(marker_brush);
    // paint rudder
    canvas.DrawSegment(center.x + LoggerData.eta[ETA_SR] * radius / 100, center.y + radius, Layout::Scale(2), Angle::Degrees(0), Angle::Degrees(360), true);
    // paint flap
    canvas.DrawSegment(center.x-radius, center.y+ LoggerData.eta[ETA_FLAP] * radius / 100, Layout::Scale(2), Angle::Degrees(0), Angle::Degrees(360), true);
}

void
InfoBoxContentControls::Update(InfoBoxData &data)
{
  if (!CommonInterface::Basic().attitude.IsBankAngleUseable() &&
      !CommonInterface::Basic().attitude.IsPitchAngleUseable()) {
    data.SetInvalid();
    return;
  }

  data.SetCustom();
}

void
UpdateInfoBoxETAhr(InfoBoxData &data)
{
  // Set Value
  data.UnsafeFormatValue(_T("%d"), LoggerData.eta[ETA_HR]);
}

void
UpdateInfoBoxETAsr(InfoBoxData &data)
{
  // Set Value
  data.UnsafeFormatValue(_T("%d"), LoggerData.eta[ETA_SR]);
}

void
UpdateInfoBoxETAqr(InfoBoxData &data)
{
  // Set Value
  data.UnsafeFormatValue(_T("%d"), LoggerData.eta[ETA_QR]);
}

void
UpdateInfoBoxMaxg(InfoBoxData &data)
{
  // Set Value
  data.UnsafeFormatValue(_T("%.1f"), 0.01*MaxValues.max_g);
}

void
UpdateInfoBoxMing(InfoBoxData &data)
{
  // Set Value
  data.UnsafeFormatValue(_T("%.1f"), 0.01*MaxValues.min_g);
}

void
UpdateInfoBoxMaxVIAS(InfoBoxData &data)
{
  // Set Value
  float speed;
  speed = sqrt(2.0*MaxValues.max_q/1.225)*3.6;

  data.UnsafeFormatValue(_T("%.0f"), speed);
}

void
UpdateInfoBoxBankAngle(InfoBoxData &data)
{
  // Set Value
  data.UnsafeFormatValue(_T("%d"), LoggerData.angle[0]/10);
}

void
UpdateInfoBoxPitchAngle(InfoBoxData &data)
{
  // Set Value
  data.UnsafeFormatValue(_T("%d"), LoggerData.angle[1]/10);
}

void
UpdateInfoBoxHeading(InfoBoxData &data)
{
  // Set Value
  data.UnsafeFormatValue(_T("%d"), (LoggerData.angle[2]+360)%360);
}

void
UpdateInfoBoxControls(InfoBoxData &data)
{
  // Set Value
  data.UnsafeFormatValue(_T("%d"), (LoggerData.angle[2]+360)%360);
}
