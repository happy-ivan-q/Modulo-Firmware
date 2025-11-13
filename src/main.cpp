#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>
#include <HIDMouse.h>

// === SPI Pins for PAW3360 ===
#define SPI_SCK   6
#define SPI_MISO  5
#define SPI_MOSI  4
#define SPI_CS    7

// PMW3360 Registers
#define REG_MOTION       0x02
#define REG_DELTA_X_L    0x03
#define REG_DELTA_X_H    0x04
#define REG_DELTA_Y_L    0x05
#define REG_DELTA_Y_H    0x06
#define REG_POWER_UP_RESET 0x3A

SPIClass spi(FSPI);
BLEHIDDevice* hid;
BLEAdvertising* advertising;
HIDMouse* mouse;

#define NOISE_THRESHOLD 2

// --- Sensor helpers ---
uint8_t readRegister(uint8_t reg) {
  digitalWrite(SPI_CS, LOW);
  spi.transfer(reg & 0x7F);
  uint8_t val = spi.transfer(0x00);
  digitalWrite(SPI_CS, HIGH);
  return val;
}

void writeRegister(uint8_t reg, uint8_t value) {
  digitalWrite(SPI_CS, LOW);
  spi.transfer(reg | 0x80);
  spi.transfer(value);
  digitalWrite(SPI_CS, HIGH);
  delayMicroseconds(50);
}

void sensorInit() {
  digitalWrite(SPI_CS, HIGH);
  writeRegister(REG_POWER_UP_RESET, 0x5A);
  delay(50);
  readRegister(REG_MOTION);
  readRegister(REG_DELTA_X_L);
  readRegister(REG_DELTA_X_H);
  readRegister(REG_DELTA_Y_L);
  readRegister(REG_DELTA_Y_H);
}

// --- BLE callbacks ---
class MyCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    advertising->stop();
  }
  void onDisconnect(BLEServer* pServer) {
    advertising->start();
  }
};

void setup() {
  Serial.begin(115200);

  // SPI sensor setup
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  sensorInit();

  // BLE HID setup
  BLEDevice::init("ESP32C6 BLE Mouse");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new MyCallbacks());

  hid = new BLEHIDDevice(server);
  mouse = new HIDMouse(hid);

  hid->manufacturer()->setValue("ESP32-C6");
  hid->pnp(0x02, 0x045E, 0x028E, 0x0100);  // Vendor/product IDs
  hid->hidInfo(0x00, 0x01);

  advertising = BLEDevice::getAdvertising();
  advertising->setAppearance(HID_MOUSE);
  advertising->addServiceUUID(hid->hidService()->getUUID());
  advertising->start();

  hid->startServices();
  BLEDevice::startAdvertising();

  Serial.println("BLE Mouse ready. Pair from your computer.");
}

void loop() {
  uint8_t motion = readRegister(REG_MOTION);
  bool motionOccurred = motion & 0x80;
  bool lifted         = motion & 0x20;

  if (!lifted && motionOccurred && mouse->isConnected()) {
    int16_t dx = (int16_t)((readRegister(REG_DELTA_X_H) << 8) | readRegister(REG_DELTA_X_L));
    int16_t dy = (int16_t)((readRegister(REG_DELTA_Y_H) << 8) | readRegister(REG_DELTA_Y_L));

    if (abs(dx) < NOISE_THRESHOLD) dx = 0;
    if (abs(dy) < NOISE_THRESHOLD) dy = 0;

    if (dx || dy) {
      mouse->move(dx, dy);
    }
  }

  delay(10);
}
 