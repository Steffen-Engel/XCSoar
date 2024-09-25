// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "InfoBoxes/Panel/Panel.hpp"
#include "InfoBoxes/Content/CIVAHMD.hpp"
#include "InfoBoxes/Panel/CIVAHMDEdit.hpp"
#include "InfoBoxes/Data.hpp"
#include "Interface.hpp"
#include "Units/Units.hpp"
#include "Formatter/UserUnits.hpp"
#include "Language/Language.hpp"

#include <tchar.h>

#include "Device/Driver/CIVAHMD.hpp"

static constexpr InfoBoxPanel panels[] = {
  { N_("Edit"), LoadCIVAHMDEditPanel },
  { nullptr, nullptr }
};

const InfoBoxPanel *
InfoBoxContentCIVAHMD::GetDialogContent() noexcept
{
  return panels;
}


void
InfoBoxContentCIVAHMD::Update(InfoBoxData &data) noexcept
{
  const NMEAInfo &basic = CommonInterface::Basic();

  // Set Value
  data.SetValueFromAltitude(basic.baro_altitude);
  data.SetValueColor((CIVAIsBeeping==0) ? (0) : (1));

  // Set Comment
  data.FmtComment(_T("Device {:02}"), CIVATargetId);
}

