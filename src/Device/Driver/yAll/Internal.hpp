/*
 * Internal.hpp
 *
 *  Created on: Feb 24, 2015
 *      Author: se
 */

#ifndef INTERNAL_yAll_HPP
#define INTERNAL_yAll_HPP

#include "Device/Driver.hpp"
#include "Device/Driver/yAll/yAll.h"


class yAllDevice : public AbstractDevice {
public:
  Port &port;
private:
  cyAll yAll;

public:
  yAllDevice(Port &_port):port(_port) {
    yAll.Open(_port);
  }
//  yAllDevice() {};
public:
  virtual bool DataReceived(std::span<const std::byte> s, struct NMEAInfo &info) noexcept override;
  virtual bool EnableNMEA(OperationEnvironment &env) override;
  virtual void OnSysTicker() override;
  virtual void LinkTimeout() override;
  void OnCalculatedUpdate(const MoreData &basic,
                          const DerivedInfo &calculated) override;

  void GetInfo(tIdent &Ident);

  void Restart();
  void CalibrateAcc();
  void CalibrateIAS();
  void CalibrateMag();
};




#endif /* INTERNAL_yAll_HPP */
