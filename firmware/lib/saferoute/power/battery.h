// Battery you can feel: voltage -> %, hours left, report pulses (`bat`),
// low-battery warning every 5 minutes, critical -> shut down (the cane still
// works as a normal cane).
#pragma once
#include "core/params.h"

namespace sr {

// 1S LiPo, resting voltage -> state of charge (%), linear between table points.
uint8_t lipoPercent(float volts);

class BatteryMonitor {
 public:
  void configure(const Params& p);
  // Call about once a second with the battery voltage (NAN if not measured).
  void update(float volts, uint32_t now);

  bool known() const { return known_; }
  float volts() const { return v_; }
  uint8_t percent() const { return pct_; }
  float hoursLeft() const;
  uint8_t reportPulses() const;          // 1-5 (20% steps) or hours left (capped at 9)
  bool low() const { return known_ && pct_ <= p_.low_battery_pct; }
  bool critical() const { return critical_; }
  // true once when the low-battery warning should play (then every 5 min)
  bool lowWarningDue(uint32_t now);

 private:
  Params p_;
  bool known_ = false;
  float v_ = 0;
  uint8_t pct_ = 0;
  bool critical_ = false;
  uint32_t crit_since_ = 0;
  bool crit_timing_ = false;
  bool warned_ = false;
  uint32_t last_warn_ = 0;
};

}  // namespace sr
