// BLE telemetry: Nordic UART Service (works with the dashboard, nRF Connect,
// Serial Bluetooth Terminal...). TX notifies telemetry lines, RX takes the same
// text commands as the USB serial port.
#pragma once
#include <stddef.h>

namespace ble {
void begin(const char* name);
bool connected();
void send(const char* line, size_t n);      // adds '\n', chunked to 20-byte notifications
bool readLine(char* out, size_t n);         // a complete command line received over RX
}  // namespace ble
