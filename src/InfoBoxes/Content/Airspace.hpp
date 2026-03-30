// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "InfoBoxes/Content/Base.hpp"

struct InfoBoxData;

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


class InfoBoxNearestAirspaceHorizontal : public InfoBoxContent
{
public:
  void Update(InfoBoxData &data) noexcept override;
  bool HandleClick() noexcept override;
};

class InfoBoxNearestAirspaceVertical : public InfoBoxContent
{
public:
  void Update(InfoBoxData &data) noexcept override;
  bool HandleClick() noexcept override;
};
