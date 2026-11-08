// Status LED: the DevKit V1's on-board blue LED (GPIO2). For sighted helpers, judges and
// debugging; the user gets haptics.
//   on = booting / alert       blip every 3 s = OK          off = paused (cane raised)
//   fast blink = calibrating   blink 2 Hz = sensor fault    blip every 1 s = not calibrated
//   double blip every 2 s = standby (tap the tip 5 times to start)
#include "hal/hal.h"

namespace hal {

static void led1(bool on) { digitalWrite(PIN_LED_SINGLE, (on ^ (LED_ACTIVE_LOW != 0)) ? HIGH : LOW); }

void StatusLed::begin() {
  pinMode(PIN_LED_SINGLE, OUTPUT);
  led1(false);                          // the pod is OFF until the first button click (main.cpp)
}

void StatusLed::on() { led1(true); }
void StatusLed::off() { led1(false); }

static bool blip(uint32_t now, uint32_t period, uint32_t on_ms) { return now % period < on_ms; }

void StatusLed::show(sr::DeviceState st, sr::Mode mode, uint32_t now) {
  using S = sr::DeviceState;
  switch (st) {
    case S::Boot:        led1(true); break;
    case S::Ok:          led1(blip(now, 3000, 80)); break;
    case S::Paused:      led1(false); break;
    case S::Alert:       led1(true); break;
    case S::Calibrating: led1(blip(now, 200, 100)); break;
    case S::Fault:       led1(blip(now, 500, 250)); break;
    case S::LowBattery:  led1(blip(now, 2000, 100) || (now % 2000 > 200 && now % 2000 < 300)); break;
    case S::NeedsCal:    led1(blip(now, 1000, 80)); break;
    case S::Off:         led1(false); break;
    case S::Standby:     led1(blip(now, 2000, 80) || (now % 2000 > 200 && now % 2000 < 280)); break;
  }
  (void)mode;
}

}  // namespace hal
