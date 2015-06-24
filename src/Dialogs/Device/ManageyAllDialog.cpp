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

#include "ManageyAllDialog.hpp"
#include "Dialogs/WidgetDialog.hpp"
#include "Widget/RowFormWidget.hpp"
#include "UIGlobals.hpp"
#include "Language/Language.hpp"
#include "Operation/MessageOperationEnvironment.hpp"
#include "Device/Driver/yAll/yAllDriver.hpp"
#include "Device/Driver/yAll/Internal.hpp"
#include "NMEA/DeviceInfo.hpp"

class ManageyAllWidget : public RowFormWidget, private ActionListener {
  /* This attribute is will be used in the next commits. */
  gcc_unused_field yAllDevice &device;
  enum Controls {
    CalibIAS,
    CalibAcc,
    CalibMag,
     Reboot,
   };



public:
  ManageyAllWidget(const DialogLook &look, yAllDevice &_device)
    :RowFormWidget(look), device(_device){}

  /* virtual methods from Widget */
  virtual void Prepare(ContainerWindow &parent, const PixelRect &rc);

private:
  /* virtual methods from ActionListener */
  virtual void OnAction(int id) override;

};



void
ManageyAllWidget::OnAction(int id)
{
  switch (id) {
  case CalibIAS:
    {
      device.CalibrateIAS();
    }
    break;
  case CalibMag:
    {
      device.CalibrateMag();
    }
    break;

  case CalibAcc:
    {
      device.CalibrateAcc();
    }
    break;

  case Reboot:
    {
      device.Restart();
    }
    break;
  }
}

void
ManageyAllWidget::Prepare(ContainerWindow &parent, const PixelRect &rc)
{
  tIdent Ident;
  device.GetInfo(Ident);

  StaticString<64> buffer;

  buffer.UnsafeFormat("%d", Ident.version);
  AddReadOnly(_("firmware version"), NULL, buffer.c_str());

  buffer.UnsafeFormat("%d", Ident.MSPVersion);
  AddReadOnly(_("MSP version"), NULL, buffer.c_str());

  buffer.UnsafeFormat("%d", Ident.multiType);
  AddReadOnly(_("quad type"), NULL, buffer.c_str());

  buffer.UnsafeFormat("%d", Ident.capabilities);
  AddReadOnly(_("capabilities"), NULL, buffer.c_str());

  AddButton(_("Calibrate Mag"), *this, CalibMag);
  AddButton(_("Calibrate Acc"), *this, CalibAcc);
  AddButton(_("Calibrate IAS"), *this, CalibIAS);
  AddButton(_("Reboot"), *this, Reboot);

}


void
ManageyAllDialog(Device &device)
{
  StaticString<64> title;
  title.Format(_T("AcroWii %s"), "version");
  WidgetDialog dialog(UIGlobals::GetDialogLook());
  dialog.CreateAuto(UIGlobals::GetMainWindow(), _T("AcroWii"),
                    new ManageyAllWidget(UIGlobals::GetDialogLook(),
                                          (yAllDevice &)device));

  dialog.AddButton(_("Close"), mrCancel);
  dialog.ShowModal();

}
