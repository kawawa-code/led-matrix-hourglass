// HT16K33 + 8x8マトリクス2枚 + LSM6DSV16X 砂時計（傾き検知で反転）
//
// 置き方（hourglass_button と同じ）
//
//          A(0,0)
//            ◇  マトリクスA
//          A(7,7)
//          B(0,0)           ← くびれ
//            ◇  マトリクスB
//          B(7,7)
//
// 配線
//   HT16K33    SDA → ESP32 21, SCL → ESP32 22  （Wire）
//   LSM6DSV16X SDA → ESP32 16, SCL → ESP32 17  （Wire1）
//   スイッチ   GPIO4 と GND の間
//
// 動作
//   ・傾き角 0度 = A が真上、180度 = B が真上
//   ・傾きが FLIP_THRESHOLD_DEG を超えると上下が入れ替わり、反対側から砂が落ちる
//   ・横倒し付近（60〜120度）では砂は止まる
//   ・スイッチ：待機中→スタート / 動作中→一時停止・再開 / 終了後→リセット

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

// =====================================
// 表示（HT16K33）
// =====================================
#define LED_SDA_PIN 21
#define LED_SCL_PIN 22

Adafruit_LEDBackpack leds = Adafruit_LEDBackpack();

const int MATRIX_A = 0;
const int MATRIX_B = 1;

// 取り付け向きの補正（テストコードで決めた値を入れる）
const uint8_t ROTATE[2] = {0, 0};

// 配線対応表（ユニバーサル基板の配線図どおり）
//   ANODE_ROW[マトリクス][行0〜7]   = つないだ HT16K33 の A 番号（A0〜A15）
//   CATHODE_COM[マトリクス][列0〜7] = つないだ HT16K33 の C 番号（C0〜C7）
// ブレッドボードの配線（順番どおり）に戻すときは次の値にする
//   ANODE_ROW   = {{0,1,2,3,4,5,6,7}, {8,9,10,11,12,13,14,15}}
//   CATHODE_COM = {{0,1,2,3,4,5,6,7}, {0,1,2,3,4,5,6,7}}
const uint8_t ANODE_ROW[2][8] = {
  {0, 2, 12, 1, 15, 11, 14, 13},   // A（上）
  {5, 3, 8, 7, 4, 10, 6, 9},       // B（下）
};
const uint8_t CATHODE_COM[2][8] = {
  {3, 7, 0, 2, 6, 1, 4, 5},        // A（上）
  {3, 0, 6, 1, 4, 2, 5, 7},        // B（下）
};

// =====================================
// 加速度センサー（LSM6DSV16X）
// =====================================
#define SENSOR_SDA_PIN 16
#define SENSOR_SCL_PIN 17

#define I2C_CLOCK 400000

#define ADDR_6A 0x6A
#define ADDR_6B 0x6B

#define REG_WHO_AM_I 0x0F
#define REG_CTRL1    0x10
#define REG_CTRL2    0x11
#define REG_CTRL6    0x15
#define REG_CTRL8    0x17
#define REG_OUTX_L_A 0x28

TwoWire& sensorWire = Wire1;
uint8_t sensorAddress = 0;

// ±8g 設定時の感度（g / LSB）
const float ACCEL_SCALE = 0.000244;

// 砂時計の縦方向（A が上のとき重力の反対向き）がセンサーのどの軸か
// シリアルに出る ax / ay / az を見て、A を上に立てたとき +1g 付近になる軸と符号を選ぶ
// 0 = X, 1 = Y, 2 = Z
const int SENSOR_AXIS = 1;
const int SENSOR_SIGN = 1;   // -1g 付近なら -1

// 上下を入れ替える角度（90より大きくする。差がヒステリシスになる）
const float FLIP_THRESHOLD_DEG = 120.0;

const float FILTER_ALPHA = 0.3;
const unsigned long SENSOR_INTERVAL_MS = 50;
const unsigned long PRINT_INTERVAL_MS = 500;

float accel[3] = {0, 0, 0};   // フィルター後（g）
bool filterInitialized = false;
float tiltDeg = 0.0;          // 0 = A が上, 180 = B が上

// =====================================
// 砂時計
// =====================================
#define BUTTON_PIN 4

const int SAND_COUNT = 64;
const unsigned long DROP_INTERVAL_MS = 1000;
const int FALL_STEP_MS = 60;

// 積もる順番（A の座標で表す。くびれ = (7,7)。B は (7-r, 7-c) に変換して使う）
int nearNeckRow[64], nearNeckCol[64];   // くびれに近い順（上の部屋の砂）
int farFirstRow[64], farFirstCol[64];   // くびれから遠い順（下の部屋の砂）

int sand[2] = {SAND_COUNT, 0};   // 各マトリクスの砂の数
int topMatrix = MATRIX_A;

enum State { IDLE, RUNNING, PAUSED, DONE };
State state = IDLE;

unsigned long lastDropTime = 0;
unsigned long lastSensorTime = 0;
unsigned long lastPrintTime = 0;



// =====================================
// 1ドット書き込み
// =====================================

void setDot(int matrix, int r, int c, bool on) {

  int pr = r;
  int pc = c;

  switch (ROTATE[matrix]) {
    case 1: pr = c;     pc = 7 - r; break;
    case 2: pr = 7 - r; pc = 7 - c; break;
    case 3: pr = 7 - c; pc = r;     break;
  }

  // 行 → A番号(ビット)、列 → C番号(displaybufferの添字)
  uint16_t bit = 1 << ANODE_ROW[matrix][pr];

  if (on) {
    leds.displaybuffer[CATHODE_COM[matrix][pc]] |= bit;
  } else {
    leds.displaybuffer[CATHODE_COM[matrix][pc]] &= ~bit;
  }
}

// くびれ基準の座標で書き込む（A はそのまま、B は 180度回して使う）
void setSand(int matrix, int r, int c, bool on) {

  if (matrix == MATRIX_B) {
    r = 7 - r;
    c = 7 - c;
  }

  setDot(matrix, r, c, on);
}



// =====================================
// 積もる順番生成
// =====================================

// r+c を sumFrom から sumTo まで step ずつ、各段は中央に近い順
void buildOrder(int sumFrom, int sumTo, int step, int* outRow, int* outCol) {

  int index = 0;

  for (int sum = sumFrom; sum != sumTo + step; sum += step) {

    int rows[8];
    int n = 0;
    for (int r = 0; r < 8; r++) {
      int c = sum - r;
      if (c >= 0 && c < 8) {
        rows[n++] = r;
      }
    }

    for (int i = 1; i < n; i++) {
      int key = rows[i];
      int j = i - 1;
      while (j >= 0 && abs(2 * rows[j] - sum) > abs(2 * key - sum)) {
        rows[j + 1] = rows[j];
        j--;
      }
      rows[j + 1] = key;
    }

    for (int i = 0; i < n; i++) {
      outRow[index] = rows[i];
      outCol[index] = sum - rows[i];
      index++;
    }
  }
}



// =====================================
// 砂を描き直す
// =====================================

void redraw() {

  int bottomMatrix = 1 - topMatrix;

  leds.clear();

  // 上の部屋：くびれ側に溜まっている
  for (int i = 0; i < sand[topMatrix]; i++) {
    setSand(topMatrix, nearNeckRow[i], nearNeckCol[i], true);
  }

  // 下の部屋：遠い側（底）から積もっている
  for (int i = 0; i < sand[bottomMatrix]; i++) {
    setSand(bottomMatrix, farFirstRow[i], farFirstCol[i], true);
  }

  leds.writeDisplay();
}



// =====================================
// 砂を1粒落とす（上の部屋 → 下の部屋）
// =====================================

void dropOneSand() {

  int bottomMatrix = 1 - topMatrix;

  // 上の部屋の一番上の砂を取る
  int topIndex = sand[topMatrix] - 1;
  setSand(topMatrix, nearNeckRow[topIndex], nearNeckCol[topIndex], false);
  sand[topMatrix]--;
  leds.writeDisplay();

  // 下の部屋の着地点
  int targetRow = farFirstRow[sand[bottomMatrix]];
  int targetCol = farFirstCol[sand[bottomMatrix]];
  int distance = 14 - (targetRow + targetCol);   // くびれからの段数

  // くびれ (7,7) から中心線を通って落ちる
  for (int i = 0; 2 * i < distance; i++) {
    setSand(bottomMatrix, 7 - i, 7 - i, true);
    leds.writeDisplay();
    delay(FALL_STEP_MS);
    setSand(bottomMatrix, 7 - i, 7 - i, false);
  }

  // 着地
  setSand(bottomMatrix, targetRow, targetCol, true);
  sand[bottomMatrix]++;
  leds.writeDisplay();
}



// =====================================
// リセット（砂をすべて今の上の部屋へ）
// =====================================

void resetHourglass() {

  sand[topMatrix] = SAND_COUNT;
  sand[1 - topMatrix] = 0;

  leds.blinkRate(HT16K33_BLINK_OFF);
  redraw();

  state = IDLE;
}



// =====================================
// センサー：レジスタ読み書き
// =====================================

bool writeReg(uint8_t reg, uint8_t value) {

  sensorWire.beginTransmission(sensorAddress);
  sensorWire.write(reg);
  sensorWire.write(value);

  return sensorWire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {

  sensorWire.beginTransmission(sensorAddress);
  sensorWire.write(reg);

  if (sensorWire.endTransmission(false) != 0) {
    return false;
  }

  if (sensorWire.requestFrom(sensorAddress, len) != len) {
    while (sensorWire.available()) {
      sensorWire.read();
    }
    return false;
  }

  for (int i = 0; i < len; i++) {
    buf[i] = sensorWire.read();
  }

  return true;
}



// =====================================
// センサー：検出・初期化
// =====================================

bool detectSensor() {

  sensorWire.beginTransmission(ADDR_6A);
  if (sensorWire.endTransmission() == 0) {
    sensorAddress = ADDR_6A;
    return true;
  }

  sensorWire.beginTransmission(ADDR_6B);
  if (sensorWire.endTransmission() == 0) {
    sensorAddress = ADDR_6B;
    return true;
  }

  return false;
}

bool initSensor() {

  uint8_t who = 0;

  if (!readRegs(REG_WHO_AM_I, &who, 1)) {
    return false;
  }

  if (who != 0x70) {
    return false;
  }

  // 秋月電子サンプルと同じ設定（加速度 ±8g）
  if (!writeReg(REG_CTRL1, 0b00001001)) return false;
  if (!writeReg(REG_CTRL2, 0b00001001)) return false;
  if (!writeReg(REG_CTRL6, 0b00000100)) return false;
  if (!writeReg(REG_CTRL8, 0b10000010)) return false;

  delay(100);

  return true;
}



// =====================================
// センサー：傾き角を求める
// =====================================

bool readTilt() {

  uint8_t buf[6];

  if (!readRegs(REG_OUTX_L_A, buf, 6)) {
    return false;
  }

  for (int i = 0; i < 3; i++) {

    int16_t raw = (int16_t)(((uint16_t)buf[i * 2 + 1] << 8) | buf[i * 2]);
    float g = raw * ACCEL_SCALE;

    // 角度ではなく加速度をフィルターする（角度だと ±180度の境目で飛ぶため）
    if (!filterInitialized) {
      accel[i] = g;
    } else {
      accel[i] += FILTER_ALPHA * (g - accel[i]);
    }
  }

  filterInitialized = true;

  float norm = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
  if (norm < 0.1) {
    return false;
  }

  // 砂時計の縦軸と「上向き」のなす角
  float cosTilt = SENSOR_SIGN * accel[SENSOR_AXIS] / norm;
  cosTilt = constrain(cosTilt, -1.0, 1.0);
  tiltDeg = acos(cosTilt) * 180.0 / PI;

  return true;
}

// 横倒し付近は砂を止める
bool isLyingDown() {
  return tiltDeg > 180.0 - FLIP_THRESHOLD_DEG && tiltDeg < FLIP_THRESHOLD_DEG;
}



// =====================================
// 上下の入れ替え
// =====================================

void updateOrientation() {

  int newTop = topMatrix;

  if (topMatrix == MATRIX_A && tiltDeg >= FLIP_THRESHOLD_DEG) {
    newTop = MATRIX_B;
  } else if (topMatrix == MATRIX_B && tiltDeg <= 180.0 - FLIP_THRESHOLD_DEG) {
    newTop = MATRIX_A;
  }

  if (newTop == topMatrix) {
    return;
  }

  topMatrix = newTop;
  Serial.println(topMatrix == MATRIX_A ? "反転：A が上" : "反転：B が上");

  // 上下が変わったので、それぞれの部屋の砂を反対側の端へ寄せて描き直す
  redraw();

  // ひっくり返したら残りの砂が落ち始める
  if (sand[topMatrix] > 0) {
    state = RUNNING;
    lastDropTime = millis();
    leds.blinkRate(HT16K33_BLINK_OFF);
  } else {
    state = DONE;
  }
}



// =====================================
// スイッチ（押した瞬間だけ true）
// =====================================

bool buttonPressed() {

  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static unsigned long changedTime = 0;

  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastReading) {
    lastReading = reading;
    changedTime = millis();
  }

  if (millis() - changedTime > 30 && reading != stableState) {
    stableState = reading;
    if (stableState == LOW) {
      return true;
    }
  }

  return false;
}



// =====================================
// setup
// =====================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // 表示
  Wire.begin(LED_SDA_PIN, LED_SCL_PIN);

  if (!leds.begin(0x70, &Wire)) {
    Serial.println("ERROR: HT16K33 not found!");
    while (true) delay(1000);
  }

  leds.setBrightness(5);

  // センサー
  sensorWire.begin(SENSOR_SDA_PIN, SENSOR_SCL_PIN);
  sensorWire.setClock(I2C_CLOCK);

  if (!detectSensor() || !initSensor()) {
    Serial.println("ERROR: LSM6DSV16X not found / init failed!");
    while (true) delay(1000);
  }

  // 砂の並び
  buildOrder(14, 0, -1, nearNeckRow, nearNeckCol);
  buildOrder(0, 14, 1, farFirstRow, farFirstCol);

  // 起動時の向きに合わせて、上になっている側に砂を入れる
  readTilt();
  topMatrix = (tiltDeg > 90.0) ? MATRIX_B : MATRIX_A;

  resetHourglass();

  Serial.println("Ready!");
}



// =====================================
// loop
// =====================================

void loop() {

  unsigned long now = millis();

  // 傾き
  if (now - lastSensorTime >= SENSOR_INTERVAL_MS) {

    lastSensorTime = now;

    if (readTilt()) {
      updateOrientation();
    } else {
      Serial.println("Sensor read error!");
    }
  }

  // 調整用の表示
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {

    lastPrintTime = now;

    Serial.printf(
      "ax %.2f  ay %.2f  az %.2f [g]  tilt %.0f deg  top %c  A %d  B %d\n",
      accel[0], accel[1], accel[2], tiltDeg,
      topMatrix == MATRIX_A ? 'A' : 'B', sand[MATRIX_A], sand[MATRIX_B]
    );
  }

  // スイッチ
  if (buttonPressed()) {

    switch (state) {
      case IDLE:
        state = RUNNING;
        lastDropTime = millis() - DROP_INTERVAL_MS;
        break;

      case RUNNING:
        state = PAUSED;
        break;

      case PAUSED:
        state = RUNNING;
        lastDropTime = millis();
        break;

      case DONE:
        resetHourglass();
        break;
    }
  }

  // 砂を落とす（横倒し中は止める）
  if (state == RUNNING && !isLyingDown() && millis() - lastDropTime >= DROP_INTERVAL_MS) {

    lastDropTime = millis();

    dropOneSand();

    if (sand[topMatrix] == 0) {
      state = DONE;
      leds.blinkRate(HT16K33_BLINK_1HZ);
    }
  }
}
