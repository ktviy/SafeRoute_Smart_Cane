#include "telemetry/ble_uart.h"
#include "config.h"

#if FEATURE_BLE
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>

namespace ble {

static const char* kService = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* kRx = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* kTx = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

static BLECharacteristic* s_tx = nullptr;
static volatile bool s_connected = false;
// ATT MTU of the current link. 23 (20-byte notifications) until the phone/laptop
// negotiates more; Chrome on Windows/Android usually does, so one telemetry line
// then goes out as one notification instead of ten.
static const uint16_t kLocalMtu = 247;
static volatile uint16_t s_mtu = 23;
static char s_rx[64];
static volatile size_t s_rx_len = 0;
static volatile bool s_rx_ready = false;

class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { s_mtu = 23; s_connected = true; }
  void onDisconnect(BLEServer* s) override {
    s_connected = false;
    s_mtu = 23;
    s->startAdvertising();
  }
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onMtuChanged(BLEServer*, esp_ble_gatts_cb_param_t* param) override { s_mtu = param->mtu.mtu; }
#endif
};

class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    const uint8_t* d = c->getData();
    size_t n = c->getLength();
    for (size_t i = 0; i < n && !s_rx_ready; i++) {
      char ch = (char)d[i];
      if (ch == '\n' || ch == '\r') {
        if (s_rx_len) { s_rx[s_rx_len] = 0; s_rx_ready = true; }
      } else if (s_rx_len < sizeof(s_rx) - 1) {
        size_t k = s_rx_len;
        s_rx[k] = ch;
        s_rx_len = k + 1;
      }
    }
  }
};

void begin(const char* name) {
  BLEDevice::init(name);
  BLEDevice::setMTU(kLocalMtu);           // largest MTU we accept; the peer may ask for less
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCb());
  BLEService* svc = server->createService(kService);
  s_tx = svc->createCharacteristic(kTx, BLECharacteristic::PROPERTY_NOTIFY);
  s_tx->addDescriptor(new BLE2902());
  BLECharacteristic* rx = svc->createCharacteristic(
      kRx, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCb());
  svc->start();
  server->getAdvertising()->addServiceUUID(kService);
  server->getAdvertising()->start();
}

bool connected() { return s_connected; }

void send(const char* line, size_t n) {
  if (!s_connected || !s_tx) return;
  static char buf[kLocalMtu];
  uint16_t mtu = s_mtu;
  size_t chunk = mtu > 23 ? mtu - 3 : 20;           // payload of one notification
  if (chunk > sizeof buf) chunk = sizeof buf;
  size_t i = 0;
  while (i <= n) {                                  // the line plus its '\n', in MTU-sized pieces
    size_t k = 0;
    while (k < chunk && i < n) buf[k++] = line[i++];
    if (k < chunk && i == n) { buf[k++] = '\n'; i++; }
    s_tx->setValue((uint8_t*)buf, k);
    s_tx->notify();
  }
}

bool readLine(char* out, size_t n) {
  if (!s_rx_ready) return false;
  strncpy(out, s_rx, n - 1);
  out[n - 1] = 0;
  s_rx_len = 0;
  s_rx_ready = false;
  return true;
}

}  // namespace ble

#else
namespace ble {
void begin(const char*) {}
bool connected() { return false; }
void send(const char*, size_t) {}
bool readLine(char*, size_t) { return false; }
}  // namespace ble
#endif
