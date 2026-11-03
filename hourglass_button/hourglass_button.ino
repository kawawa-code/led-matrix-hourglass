// HT16K33 + 8x8マトリクス2枚 砂時計（スイッチ式）
//
// 置き方
//   2枚を45度傾けて「ひし形」にし、角同士を合わせて縦に並べる。
//
//          A(0,0)           ← 上の角
//            ◇  マトリクスA（上）
//          A(7,7)
//          B(0,0)           ← くびれ（Aの下の角とBの上の角が接する）
//            ◇  マトリクスB（下）
//          B(7,7)           ← 下の角
//
//   ひし形にすると「r + c」が大きいほど下にある。
//   砂は r + c の大きい（低い）ところから積もる。
//
// 操作（スイッチは GPIO4 と GND の間。内部プルアップを使用）
//   待機中に押す   → スタート（1秒に1粒ずつ A → B へ落ちる）
//   動作中に押す   → 一時停止 / 再開
//   終了後に押す   → リセット（砂がAに戻る）

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

Adafruit_LEDBackpack leds = Adafruit_LEDBackpack();

const int MATRIX_A = 0;  // 上
const int MATRIX_B = 1;  // 下

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

const int BUTTON_PIN = 4;

const int SAND_COUNT = 64;                   // 砂の数（1〜64）
const unsigned long DROP_INTERVAL_MS = 1000; // 1粒落ちる間隔
const int FALL_STEP_MS = 60;                 // 落下アニメーションの速さ

// 積もる順番（下から順）。A への積み方と B への積み方は同じ並びを使う
int orderRow[64];
int orderCol[64];

enum State { IDLE, RUNNING, PAUSED, DONE };
State state = IDLE;

int movedCount = 0;             // B に移った砂の数
unsigned long lastDropTime = 0;



// =======================
// 1ドット書き込み
// =======================

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



// =======================
// 積もる順番生成
// =======================

void buildSandOrder() {

  int index = 0;

  // 下の角 (r+c=14) から上の角 (r+c=0) へ、ひし形の横一列ずつ
  for (int sum = 14; sum >= 0; sum--) {

    // この段に並ぶ行番号を集める
    int rows[8];
    int n = 0;
    for (int r = 0; r < 8; r++) {
      int c = sum - r;
      if (c >= 0 && c < 8) {
        rows[n++] = r;
      }
    }

    // 段の中央に近い順に並べ替え（山なりに積もる）
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
      orderRow[index] = rows[i];
      orderCol[index] = sum - rows[i];
      index++;
    }
  }
}



// =======================
// リセット（砂をすべて A に戻す）
// =======================

void resetHourglass() {

  leds.clear();

  for (int i = 0; i < SAND_COUNT; i++) {
    setDot(MATRIX_A, orderRow[i], orderCol[i], true);
  }

  leds.blinkRate(HT16K33_BLINK_OFF);
  leds.writeDisplay();

  movedCount = 0;
  state = IDLE;
}



// =======================
// 砂を1粒落とす
// =======================

void dropOneSand() {

  // A の一番上の砂を取る（積んだ順の逆）
  int topIndex = SAND_COUNT - 1 - movedCount;
  setDot(MATRIX_A, orderRow[topIndex], orderCol[topIndex], false);
  leds.writeDisplay();

  // B の着地点（空いている一番下）
  int targetRow = orderRow[movedCount];
  int targetCol = orderCol[movedCount];
  int targetSum = targetRow + targetCol;

  // くびれ B(0,0) から中心線 (i,i) を通って落ちる
  // 中心線上で着地点より上の段 (2i < targetSum) はまだ空いている
  for (int i = 0; 2 * i < targetSum; i++) {
    setDot(MATRIX_B, i, i, true);
    leds.writeDisplay();
    delay(FALL_STEP_MS);
    setDot(MATRIX_B, i, i, false);
  }

  // 着地
  setDot(MATRIX_B, targetRow, targetCol, true);
  leds.writeDisplay();

  movedCount++;
}



// =======================
// スイッチ（押した瞬間だけ true）
// =======================

bool buttonPressed() {

  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static unsigned long changedTime = 0;

  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastReading) {
    lastReading = reading;
    changedTime = millis();
  }

  // 30ms 変化がなければ確定（チャタリング対策）
  if (millis() - changedTime > 30 && reading != stableState) {
    stableState = reading;
    if (stableState == LOW) {
      return true;
    }
  }

  return false;
}



// =======================
// 初期化
// =======================

void setup() {

  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(21, 22);

  if (!leds.begin(0x70)) {
    Serial.println("HT16K33 が見つかりません。配線とアドレスを確認してください。");
    while (true) delay(1000);
  }

  leds.setBrightness(5);  // 0〜15

  buildSandOrder();
  resetHourglass();
}



// =======================
// メイン
// =======================

void loop() {

  if (buttonPressed()) {

    switch (state) {
      case IDLE:
        state = RUNNING;
        lastDropTime = millis() - DROP_INTERVAL_MS;  // すぐ1粒目を落とす
        Serial.println("スタート");
        break;

      case RUNNING:
        state = PAUSED;
        Serial.println("一時停止");
        break;

      case PAUSED:
        state = RUNNING;
        lastDropTime = millis();
        Serial.println("再開");
        break;

      case DONE:
        resetHourglass();
        Serial.println("リセット");
        break;
    }
  }

  if (state == RUNNING && millis() - lastDropTime >= DROP_INTERVAL_MS) {

    lastDropTime = millis();

    dropOneSand();

    if (movedCount >= SAND_COUNT) {
      state = DONE;
      leds.blinkRate(HT16K33_BLINK_1HZ);  // 終了したら点滅
      Serial.println("終了");
    }
  }
}
