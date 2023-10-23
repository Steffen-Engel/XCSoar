// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "HorizonRenderer.hpp"
#include "RadarRenderer.hpp"
#include "ui/canvas/Canvas.hpp"
#include "Screen/Layout.hpp"
#include "Look/HorizonLook.hpp"
#include "NMEA/Attitude.hpp"
#include "Math/Constants.hpp"
#include "Math/Util.hpp"

#include <algorithm>

void
DrawMe(Canvas &canvas, const PixelRect &rc,
                      const HorizonLook &look,
                      const AttitudeState &attitude);


void
DrawMe(Canvas &canvas, const PixelRect &rc,
                      const HorizonLook &look,
                      const AttitudeState &attitude)
{
  static Pen angle_pen;
  static bool inized = false;
  if (!inized)
  {
    angle_pen.Create(Layout::Scale(1), COLOR_LIGHT_GRAY);
    inized = true;
  }
  /*
  This feature of having a backup artificial horizon based on inferred
  orientation from GPS and vario data is useful, and reasonably well
  tested, but has the issue of potentially invalidating use of XCSoar in
  FAI contests due to rule ref Annex A to Section 3 (2010 Edition) 4.1.2
  "No instruments permitting pilots to fly without visual reference to
  the ground may be carried on board, even if made unserviceable."  The
  quality of XCSoar's pseudo-AH is arguably good enough that this
  violates the rule.  We need to seek clarification as to whether this
  is the case or not.
  */

  const auto center = rc.GetCenter();

  const int radius = std::min(rc.GetWidth(), rc.GetHeight()) / 2
    - Layout::Scale(1);

  auto bank_degrees = attitude.bank_angle_available
    ? attitude.bank_angle.Degrees()
    : 0.;

  auto pitch_degrees = attitude.pitch_angle_available
    ? attitude.pitch_angle.Degrees()
    : 0.;

  auto phi = std::clamp(bank_degrees, -89., 89.);
  phi = bank_degrees;
  auto alpha = Angle::acos(std::clamp(pitch_degrees / 50,
                                 -1., 1.));
  auto sphi = Angle::HalfCircle() - Angle::Degrees(phi);
  auto alpha1 = sphi - alpha;
  auto alpha2 = sphi + alpha;

  phi = bank_degrees;

  // draw sky part
  canvas.Select(look.sky_pen);
  canvas.Select(look.sky_brush);
  canvas.DrawSegment(center, radius, alpha2, alpha1, true);

  // draw ground part
  canvas.Select(look.terrain_pen);
  canvas.Select(look.terrain_brush);
  canvas.DrawSegment(center, radius, alpha1, alpha2, true);

  // draw aircraft symbol
  canvas.Select(look.aircraft_pen);
  canvas.DrawLine({center.x + radius / 2, center.y}, {center.x - radius / 2, center.y});
  canvas.DrawLine({center.x, center.y - radius / 4}, {center.x, center.y});

  // draw 45 degree dash marks
  const int rr2p = uround(radius * M_SQRT1_2) + Layout::Scale(1);
  const int rr2n = rr2p - Layout::Scale(2);
  canvas.DrawLine({center.x + rr2p, center.y - rr2p},
              {center.x + rr2n, center.y - rr2n});
  canvas.DrawLine({center.x - rr2p, center.y - rr2p},
              {center.x - rr2n, center.y - rr2n});


  // draw pitch-angle lines in horizon
  canvas.Select(angle_pen);
  int angle[] = {-45, -30, -20, -10, 10, 20, 30, 45, 0};
  double len[] = {double(0.2), double(0.15), double(0.15), double(0.15), double(0.15), double(0.15), double(0.15), double(0.25)};
  for (int count = 0; angle[count] != 0; count++)
  {
    double x_pos0 = double(center.x) + (pitch_degrees+double(angle[count]))*double(radius)/double(50.0) * sin(bank_degrees/180*M_PI);
    double x_pos1 = x_pos0 - radius * double(len[count]) * cos(bank_degrees/180*M_PI);
    double x_pos2 = x_pos0 + radius * double(len[count]) * cos(bank_degrees/180*M_PI);

    double y_pos0 = double(center.y) + (pitch_degrees+double(angle[count]))*double(radius)/50 * cos(bank_degrees/180*M_PI);
    double y_pos1 = y_pos0 + double(radius) * sin(bank_degrees/180*M_PI)*len[count];
    double y_pos2 = y_pos0 - double(radius) * sin(bank_degrees/180*M_PI)*len[count];
    canvas.DrawLine({(int)x_pos1, (int)y_pos1}, {(int)x_pos2, (int)y_pos2});
  }
}



void
HorizonRenderer::Draw(Canvas &canvas, const PixelRect &rc,
                      const HorizonLook &look,
                      const AttitudeState &attitude)
{
  DrawMe(canvas, rc, look, attitude);
  return;
  /*
  This feature of having a backup artificial horizon based on inferred
  orientation from GPS and vario data is useful, and reasonably well
  tested, but has the issue of potentially invalidating use of XCSoar in
  FAI contests due to rule ref Annex A to Section 3 (2010 Edition) 4.1.2
  "No instruments permitting pilots to fly without visual reference to
  the ground may be carried on board, even if made unserviceable."  The
  quality of XCSoar's pseudo-AH is arguably good enough that this
  violates the rule.  We need to seek clarification as to whether this
  is the case or not.
  */

  RadarRenderer radar_renderer{Layout::Scale(1U)};
  radar_renderer.UpdateLayout(rc);

  const auto center = radar_renderer.GetCenter();
  const int radius = radar_renderer.GetRadius();

  auto bank_degrees = attitude.bank_angle_available
    ? attitude.bank_angle.Degrees()
    : 0.;

  auto pitch_degrees = attitude.pitch_angle_available
    ? attitude.pitch_angle.Degrees()
    : 0.;

  auto cosine_ratio = pitch_degrees / 50;
  auto alpha = Angle::acos(std::clamp(cosine_ratio, -1., 1.));
  auto sphi = Angle::HalfCircle() - Angle::Degrees(bank_degrees);
  auto alpha1 = sphi - alpha;
  auto alpha2 = sphi + alpha;

  // draw sky part
  if (cosine_ratio > -1 ) { // when less than -1 then the sky is not visible
    canvas.Select(look.sky_pen);
    canvas.Select(look.sky_brush);
    canvas.DrawSegment(center, radius, alpha2, alpha1, true);
  }

  // draw ground part
  if (cosine_ratio < 1) { // when greater than 1 then the ground is not visible
    canvas.Select(look.terrain_pen);
    canvas.Select(look.terrain_brush);
    canvas.DrawSegment(center, radius, alpha1, alpha2, true);
  }
  
  // draw aircraft symbol
  canvas.Select(look.aircraft_pen);
  canvas.DrawLine({center.x + radius / 2, center.y}, {center.x - radius / 2, center.y});
  canvas.DrawLine({center.x, center.y - radius / 4}, {center.x, center.y});

  // draw 45 degree dash marks
  const int rr2p = uround(radius * M_SQRT1_2) + Layout::Scale(1);
  const int rr2n = rr2p - Layout::Scale(2);
  canvas.DrawLine({center.x + rr2p, center.y - rr2p},
                  {center.x + rr2n, center.y - rr2n});
  canvas.DrawLine({center.x - rr2p, center.y - rr2p},
                  {center.x - rr2n, center.y - rr2n});
}
