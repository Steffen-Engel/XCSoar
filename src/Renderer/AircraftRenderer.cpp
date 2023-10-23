// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "AircraftRenderer.hpp"
#include "RotatedPolygonRenderer.hpp"
#include "ui/canvas/Canvas.hpp"
#include "Look/AircraftLook.hpp"
#include "MapSettings.hpp"
#include "Asset.hpp"
#include "Math/Angle.hpp"
#include "Screen/Layout.hpp"
#include "util/Macros.hpp"

static void
DrawMirroredPolygon(std::span<const BulkPixelPoint> src,
                    Canvas &canvas, const Angle angle,
                    const PixelPoint pos)
{
  std::array<BulkPixelPoint, 64> dst;
  assert(2 * src.size() <= dst.size());

  std::copy(src.begin(), src.end(), dst.begin());
  for (std::size_t i = 0; i < src.size(); ++i) {
    dst[2 * src.size() - i - 1].x = -dst[i].x;
    dst[2 * src.size() - i - 1].y = dst[i].y;
  }
#ifdef ENABLE_OPENGL
  CanvasRotateShift rotate_shift(pos, angle, Layout::Scale(0.5f));
#else
  PolygonRotateShift({dst.data(), 2 * src.size()}, pos, angle, Layout::Scale(50U));
#endif
  canvas.DrawPolygon(dst.data(), 2 * src.size());
}

static void
DrawDetailedAircraft(Canvas &canvas, bool inverse,
                     const AircraftLook &look,
                     const Angle angle,
                     const PixelPoint aircraft_pos)
{
  {
    static constexpr BulkPixelPoint Aircraft[] = {
      {0, -10},
      {-2, -7},
      {-2, -2},
      {-16, -2},
      {-32, -1},
      {-32, 2},
      {-1, 3},
      {-1, 15},
      {-3, 15},
      {-5, 17},
      {-5, 18},
      {0, 18},
    };

    if (!inverse) {
      canvas.SelectWhiteBrush();
      canvas.Select(look.aircraft_pen);
    } else {
      canvas.SelectBlackBrush();
      canvas.SelectWhitePen();
    }

    DrawMirroredPolygon(Aircraft, canvas, angle, aircraft_pos);
  }

  {
    static constexpr BulkPixelPoint Canopy[] = {
      {0, -7},
      {-1, -7},
      {-1, -2},
      {0, -1},
    };

    canvas.Select(look.canopy_pen);
    canvas.Select(look.canopy_brush);
    DrawMirroredPolygon(Canopy, canvas, angle, aircraft_pos);
  }
}


static void
DrawSimpleAircraft(Canvas &canvas, const AircraftLook &look,
                   const Angle angle,
                   const PixelPoint aircraft_pos, bool large)
{
  static constexpr BulkPixelPoint AircraftLarge[] = {
    {1, -7},
    {1, -1},
    {17, -1},
    {17, 1},
    {1, 1},
    {1, 10},
    {5, 10},
    {5, 12},
    {-5, 12},
    {-5, 10},
    {-1, 10},
    {-1, 1},
    {-17, 1},
    {-17, -1},
    {-1, -1},
    {-1, -7},
  };

  static constexpr BulkPixelPoint AircraftSmall[] = {
    {1, -5},
    {1, 0},
    {14, 0},
    {14, 1},
    {1, 1},
    {1, 8},
    {4, 8},
    {4, 9},
    {-3, 9},
    {-3, 8},
    {0, 8},
    {0, 1},
    {-13, 1},
    {-13, 0},
    {0, 0},
    {0, -5},
   };

  static constexpr unsigned AIRCRAFT_POINTS_LARGE = ARRAY_SIZE(AircraftLarge);
  static constexpr unsigned AIRCRAFT_POINTS_SMALL = ARRAY_SIZE(AircraftSmall);

  const auto *Aircraft = large ? AircraftLarge : AircraftSmall;
  const std::size_t AircraftPoints = large ?
                                  AIRCRAFT_POINTS_LARGE : AIRCRAFT_POINTS_SMALL;

  const RotatedPolygonRenderer renderer({Aircraft, AircraftPoints},
                                        aircraft_pos, angle);

  canvas.SelectHollowBrush();
  canvas.Select(look.aircraft_simple2_pen);
  renderer.Draw(canvas, 0, AircraftPoints);
  canvas.SelectBlackBrush();
  canvas.Select(look.aircraft_simple1_pen);
  renderer.Draw(canvas, 0, AircraftPoints);
}

static void
DrawHangGlider(Canvas &canvas, [[maybe_unused]] const AircraftLook &look,
               const Angle angle, const PixelPoint aircraft_pos, bool inverse)
{
  static constexpr BulkPixelPoint aircraft[] = {
    {1, -3},
    {7, 0},
    {13, 4},
    {13, 6},
    {6, 3},
    {1, 2},
    {-1, 2},
    {-6, 3},
    {-13, 6},
    {-13, 4},
    {-7, 0},
    {-1, -3},
   };

  if (inverse) {
    canvas.SelectBlackBrush();
    canvas.SelectWhitePen();
  } else {
    canvas.SelectWhiteBrush();
    canvas.SelectBlackPen();
  }

  const RotatedPolygonRenderer renderer(aircraft,
                                        aircraft_pos, angle);
  renderer.Draw(canvas, 0, ARRAY_SIZE(aircraft));
}

static void
DrawParaGlider(Canvas &canvas, [[maybe_unused]] const AircraftLook &look,
               const Angle angle, const PixelPoint aircraft_pos, bool inverse)
{
  static constexpr BulkPixelPoint aircraft[] = {
    // Wing
    {-5, 3},
    {-11, 2},
    {-14, 1},
    {-14, -1},
    {-11, -2},
    {-5, -3},
    {5, -3},
    {11, -2},
    {14, -1},
    {14, 1},
    {11, 2},
    {5, 3},

    // Arrow on wing
    {2, 3},
    {-2, 3},
    {0, -3},
   };

  const RotatedPolygonRenderer renderer(aircraft,
                                        aircraft_pos, angle);

  if (inverse) {
    canvas.SelectBlackBrush();
    canvas.SelectWhitePen();
  } else {
    canvas.SelectWhiteBrush();
    canvas.SelectBlackPen();
  }

  renderer.Draw(canvas, 0, ARRAY_SIZE(aircraft) - 3);

  canvas.SelectNullPen();
  if (inverse)
    canvas.SelectWhiteBrush();
  else
    canvas.SelectBlackBrush();

  renderer.Draw(canvas, ARRAY_SIZE(aircraft) - 3, 3);
}


static void
DrawArrow(Canvas &canvas, [[maybe_unused]] const AircraftLook &look,
               const Angle angle, const PixelPoint aircraft_pos, [[maybe_unused]] bool inverse)
{
  static constexpr BulkPixelPoint aircraft[] = {
    // right
    {0, 5},
    {-5, 8},
    {0, -8},


    // left
    {0, -8},
    {5, 8},
    {0, 5},
  };

  const RotatedPolygonRenderer renderer(aircraft,
                                        aircraft_pos, angle);

  Pen pen1, pen2, pen3;
  Brush brush1, brush2;
  brush1.Create(Color(0x8f, 0x00, 0x00));
  pen1.Create(2, Color(0x00, 0x00, 0x00));
  brush2.Create(Color(0xf8, 0x00, 0x00));
  pen2.Create(2, Color(0x00, 0x00, 0x00));
  pen3.Create(3, Color(0xef, 0x00, 0x00));


  canvas.Select(pen3);
  canvas.SelectHollowBrush();
  int radius = Layout::Scale(12);
  canvas.DrawCircle(aircraft_pos, radius-1);
  canvas.DrawCircle(aircraft_pos, radius);
  canvas.DrawCircle(aircraft_pos, radius+1);

  canvas.Select(brush1);
  canvas.Select(pen1);

  renderer.Draw(canvas, 0, ARRAY_SIZE(aircraft) - 3);

  canvas.Select(brush2);
  canvas.SelectNullPen();

  renderer.Draw(canvas, ARRAY_SIZE(aircraft) - 3, 3);

}


void
AircraftRenderer::Draw(Canvas &canvas, const MapSettings &settings_map,
                       const AircraftLook &look,
                       const Angle angle, const PixelPoint aircraft_pos)
{
  const bool inverse = IsDithered() || !settings_map.terrain.enable;

  switch (settings_map.aircraft_symbol) {
  case AircraftSymbol::DETAILED:
    DrawDetailedAircraft(canvas, inverse,
                         look, angle, aircraft_pos);
    break;

  case AircraftSymbol::SIMPLE_LARGE:
    DrawSimpleAircraft(canvas, look, angle, aircraft_pos, true);
    break;

  case AircraftSymbol::SIMPLE:
    DrawSimpleAircraft(canvas, look, angle, aircraft_pos, false);
    break;

  case AircraftSymbol::HANGGLIDER:
    DrawHangGlider(canvas, look, angle, aircraft_pos,
                   inverse);
    break;

  case AircraftSymbol::PARAGLIDER:
    DrawParaGlider(canvas, look, angle, aircraft_pos,
                   inverse);
    break;
  case AircraftSymbol::ARROW:
    DrawArrow(canvas, look, angle, aircraft_pos,
                   inverse);
    break;
  }
}
