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

#include "time/PeriodClock.hpp"

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
  data.FmtComment("Device {:02}", CIVATargetId);
}

extern PeriodClock last_CIVA_Receive_time;

extern long int CIVA_Count;
void
UpdateInfoBoxCIVAHMDTime(InfoBoxData &data) noexcept
{
	  const NMEAInfo &basic = CommonInterface::Basic();

	  if (!basic.time_available) {
	    data.SetInvalid();
	    return;
	  }

	  // Set Value
	  const BrokenDateTime t = basic.date_time_utc;


	  // Set Comment
	  //long long int timer = last_CIVA_Receive_time.Elapsed();

	  const auto dt = last_CIVA_Receive_time.Elapsed();
	  long int timer = std::chrono::round<std::chrono::milliseconds>(dt).count();
	  data.FmtValue("{:3}", CIVA_Count);

	  //data.FmtComment("{:02}", timer);
}
