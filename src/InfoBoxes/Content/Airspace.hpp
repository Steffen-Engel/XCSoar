// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "InfoBoxes/Content/Base.hpp"

struct InfoBoxData;

void
UpdateInfoBoxNearestAirspaceHorizontal(InfoBoxData &data) noexcept;

void

UpdateInfoBoxNearestAirspaceVertical(InfoBoxData &data) noexcept;

class InfoBoxContentAirspaces_Distance : public InfoBoxContent
{
 public:
  virtual void Update(InfoBoxData &data) noexcept override;
  virtual void OnCustomPaint(Canvas &canvas, const PixelRect &rc) noexcept override;
};

class InfoBoxContentAirspaces_Name : public InfoBoxContent
{
 public:
  virtual void Update(InfoBoxData &data) noexcept override;
  virtual void OnCustomPaint(Canvas &canvas, const PixelRect &rc) noexcept override;
};

class InfoBoxContentAirspaces_Altitude : public InfoBoxContent
{
 public:
  virtual void Update(InfoBoxData &data) noexcept override;
  virtual void OnCustomPaint(Canvas &canvas, const PixelRect &rc) noexcept override;
};
