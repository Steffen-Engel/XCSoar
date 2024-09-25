// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "CIVAHMDEdit.hpp"
#include "Look/DialogLook.hpp"
#include "Widget/OffsetButtonsWidget.hpp"
#include "Formatter/UserUnits.hpp"
#include "ActionInterface.hpp"
#include "UIGlobals.hpp"
#include "Device/Driver/CIVAHMD.hpp"


class CIVAOffsetButtons final : public OffsetButtonsWidget {
public:
  CIVAOffsetButtons(bool active_freq) noexcept
    :OffsetButtonsWidget(UIGlobals::GetDialogLook().button, _T("%.0f"), 1, 10), set_active_freq(active_freq) {}

protected:
  /* virtual methods from OffsetButtonsWidget */
  void OnOffset(double offset) noexcept override;

private:
  bool set_active_freq;
};

void
CIVAOffsetButtons::OnOffset(double offset) noexcept
{
  if(set_active_freq) {
  }

  CIVATargetId += (int)offset;
  if (CIVATargetId < 0)
    CIVATargetId = 0;

}

std::unique_ptr<Widget>
LoadCIVAHMDEditPanel([[maybe_unused]] unsigned id)
{
  return std::make_unique<CIVAOffsetButtons>(true);
}

