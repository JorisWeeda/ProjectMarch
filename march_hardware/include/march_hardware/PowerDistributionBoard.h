// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_POWERDISTRIBUTIONBOARD_H
#define MARCH_HARDWARE_POWERDISTRIBUTIONBOARD_H
#include "march_hardware/BootShutdownOffsets.h"
#include "march_hardware/EtherCAT/slave.h"
#include "march_hardware/HighVoltage.h"
#include "march_hardware/LowVoltage.h"
#include "march_hardware/NetDriverOffsets.h"
#include "march_hardware/NetMonitorOffsets.h"

namespace march
{
class PowerDistributionBoard : public Slave
{
private:
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  BootShutdownOffsets bootShutdownOffsets;
  HighVoltage highVoltage;
  LowVoltage lowVoltage;
  bool masterOnlineToggle;

public:
  PowerDistributionBoard(Slave slave, NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets,
                         BootShutdownOffsets bootShutdownOffsets);

  float getPowerDistributionBoardCurrent();
  bool getMasterShutdownRequested();
  void setMasterOnline();
  void setMasterShutDownAllowed(bool isAllowed);

  HighVoltage getHighVoltage();
  LowVoltage getLowVoltage();

  /** @brief Override comparison operator */
  friend bool operator==(const PowerDistributionBoard& lhs, const PowerDistributionBoard& rhs)
  {
    return lhs.getSlaveIndex() == rhs.getSlaveIndex() && lhs.netMonitoringOffsets == rhs.netMonitoringOffsets &&
           lhs.netDriverOffsets == rhs.netDriverOffsets && lhs.lowVoltage == rhs.lowVoltage &&
           lhs.highVoltage == rhs.highVoltage;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const PowerDistributionBoard& powerDistributionBoard)
  {
    return os << "PowerDistributionBoard(slaveIndex: " << powerDistributionBoard.getSlaveIndex() << ", "
              << "masterOnlineToggle: " << powerDistributionBoard.masterOnlineToggle << ", "
              << powerDistributionBoard.highVoltage << ", " << powerDistributionBoard.lowVoltage << ")";
  }
};
}  // namespace march
#endif  // MARCH_HARDWARE_POWERDISTRIBUTIONBOARD_H
