// [130950]6軸IMUセンサーモジュール AE-LSM6DSV16X
// https://akizukidenshi.com/catalog/g/g130950/
// 本プログラムは上記モジュールを簡易的に動作確認するプログラムです。
//
// ======== 注意！ ========
// 本モジュールは5V系のマイコンボード（Arduino Unoなど）ではお使いいただけません！
// Raspberry Pi PicoやEPS32など3.3V系のマイコンボードに付属のQwiicケーブルを使い以下のように繋いでください。
//  黒 : GND
//  赤 : 3.3V (5V不可)
//  青 : SDA
//  黄 : SCL
// 本サンプルプログラムはI2Cで通信するため、J2（「6B」刻印近く）もしくはJ3（「6A」刻印近く）のジャンパーパターンをハンダで短絡させてください。
// また、プルアップ抵抗を外部で用意しない場合は、J1もハンダで短絡させてください。
// 
// (株）秋月電子通商 2025-12-22


#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// =====================================
// ESP32 I2C設定
// =====================================
#define SDA_PIN 16
#define SCL_PIN 17

#define I2C_CLOCK 400000

// =====================================
// LSM6DSV16X設定
// =====================================
#define ADDR_6A 0x6A
#define ADDR_6B 0x6B

#define REG_WHO_AM_I 0x0F
#define REG_CTRL1    0x10
#define REG_CTRL2    0x11
#define REG_CTRL6    0x15
#define REG_CTRL8    0x17
#define REG_OUTX_L_G 0x22

uint8_t sensorAddress = 0;

// =====================================
// 角度設定
// =====================================
float roll = 0.0;
float pitch = 0.0;

float filteredRoll = 0.0;
float filteredPitch = 0.0;

bool filterInitialized = false;

// フィルター設定
const float FILTER_ALPHA = 0.12;

// 取得間隔：1000ms = 1秒
const unsigned long UPDATE_INTERVAL = 1000;

unsigned long lastUpdate = 0;

// =====================================
// レジスタ書き込み
// =====================================
bool writeReg(uint8_t reg, uint8_t value) {

  Wire.beginTransmission(sensorAddress);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

// =====================================
// レジスタ読み込み
// =====================================
bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {

  Wire.beginTransmission(sensorAddress);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(sensorAddress, len) != len) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (int i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }

  return true;
}

// =====================================
// センサー検出
// =====================================
bool detectSensor() {

  Wire.beginTransmission(ADDR_6A);

  if (Wire.endTransmission() == 0) {
    sensorAddress = ADDR_6A;
    return true;
  }

  Wire.beginTransmission(ADDR_6B);

  if (Wire.endTransmission() == 0) {
    sensorAddress = ADDR_6B;
    return true;
  }

  return false;
}

// =====================================
// センサー初期化
// =====================================
bool initSensor() {

  uint8_t who = 0;

  if (!readRegs(REG_WHO_AM_I, &who, 1)) {
    return false;
  }

  Serial.print("WHO_AM_I: 0x");
  Serial.println(who, HEX);

  if (who != 0x70) {
    return false;
  }

  // 秋月電子サンプルと同じ設定
  if (!writeReg(REG_CTRL1, 0b00001001)) return false;
  if (!writeReg(REG_CTRL2, 0b00001001)) return false;
  if (!writeReg(REG_CTRL6, 0b00000100)) return false;
  if (!writeReg(REG_CTRL8, 0b10000010)) return false;

  delay(100);

  return true;
}

// =====================================
// 角度取得
// =====================================
bool readAngle() {

  uint8_t buf[12];

  if (!readRegs(REG_OUTX_L_G, buf, 12)) {
    return false;
  }

  int16_t data[6];

  for (int i = 0; i < 6; i++) {

    uint16_t raw =
      ((uint16_t)buf[i * 2 + 1] << 8) |
      buf[i * 2];

    data[i] = (int16_t)raw;
  }

  // 加速度データ
  float ax = data[3];
  float ay = data[4];
  float az = data[5];

  // Roll角度
  roll = atan2(ay, az) * 180.0 / PI;

  // Pitch角度
  pitch = atan2(
    -ax,
    sqrt(ay * ay + az * az)
  ) * 180.0 / PI;

  // ローパスフィルター
  if (!filterInitialized) {

    filteredRoll = roll;
    filteredPitch = pitch;

    filterInitialized = true;

  } else {

    filteredRoll +=
      FILTER_ALPHA * (roll - filteredRoll);

    filteredPitch +=
      FILTER_ALPHA * (pitch - filteredPitch);
  }

  return true;
}

// =====================================
// 角度表示
// =====================================
void printAngle() {

  Serial.printf(
    "Roll: %.2f deg, Pitch: %.2f deg\n",
    filteredRoll,
    filteredPitch
  );
}

// =====================================
// setup
// =====================================
void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("LSM6DSV16X Starting...");

  // I2C開始
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK);

  // センサー検出
  if (!detectSensor()) {

    Serial.println("ERROR: Sensor not found!");

    while (1) {
      delay(1000);
    }
  }

  Serial.print("Sensor address: 0x");
  Serial.println(sensorAddress, HEX);

  // センサー初期化
  if (!initSensor()) {

    Serial.println("ERROR: Initialization failed!");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("Sensor Ready!");
  Serial.println("Roll / Pitch");
  Serial.println("------------------------------");

  lastUpdate = millis();
}

// =====================================
// loop
// =====================================
void loop() {

  unsigned long now = millis();

  if (now - lastUpdate >= UPDATE_INTERVAL) {

    lastUpdate = now;

    if (readAngle()) {

      printAngle();

    } else {

      Serial.println("Sensor read error!");
    }
  }
}
