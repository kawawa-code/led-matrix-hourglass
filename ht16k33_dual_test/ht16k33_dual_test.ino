// HT16K33 + 8x8マトリクス2枚 テストコード
//
// 配線
//   マトリクスのアノード(行) → HT16K33 A0〜A15、カソード(列) → C0〜C7
//   どのピンをどこにつないだかは ANODE_ROW / CATHODE_COM の表に書く
//   HT16K33 SDA → ESP32 21, SCL → ESP32 22
//
// テスト内容（ループで繰り返す）
//   1. 全点灯        … 128個すべて光るか
//   2. A だけ / B だけ … 上下のマトリクスの振り分けが正しいか
//   3. (0,0) を点滅   … 各マトリクスの基準位置（向き）の確認
//   4. 1ドットずつ走査 … 抜けや順番違いがないか（シリアルに座標を表示）

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

Adafruit_LEDBackpack leds = Adafruit_LEDBackpack();

const int MATRIX_A = 0;  // 上
const int MATRIX_B = 1;  // 下

// 取り付け向きの補正（0〜3 = 時計回りに 0/90/180/270 度）
// 砂時計本体でも同じ値を使う。(0,0) が両方とも「上の角」に来るように調整する。
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

void fillMatrix(int matrix, bool on) {
  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      setDot(matrix, r, c, on);
    }
  }
}



// =======================
// 初期化
// =======================

void setup() {

  Serial.begin(115200);

  Wire.begin(21, 22);

  if (!leds.begin(0x70)) {
    Serial.println("HT16K33 が見つかりません。配線とアドレスを確認してください。");
    while (true) delay(1000);
  }

  leds.setBrightness(5);  // 0〜15
  leds.clear();
  leds.writeDisplay();
}



// =======================
// メイン
// =======================

void loop() {

  // 1. 全点灯
  Serial.println("[1] 全点灯");
  fillMatrix(MATRIX_A, true);
  fillMatrix(MATRIX_B, true);
  leds.writeDisplay();
  delay(2000);

  // 2. A だけ → B だけ
  Serial.println("[2] A(上)だけ点灯");
  leds.clear();
  fillMatrix(MATRIX_A, true);
  leds.writeDisplay();
  delay(1000);

  Serial.println("[2] B(下)だけ点灯");
  leds.clear();
  fillMatrix(MATRIX_B, true);
  leds.writeDisplay();
  delay(1000);

  // 3. (0,0) を点滅
  Serial.println("[3] A と B の (0,0) を点滅");
  for (int i = 0; i < 3; i++) {
    leds.clear();
    setDot(MATRIX_A, 0, 0, true);
    setDot(MATRIX_B, 0, 0, true);
    leds.writeDisplay();
    delay(400);

    leds.clear();
    leds.writeDisplay();
    delay(400);
  }

  // 4. 1ドットずつ走査（A → B、行ごとに左から右）
  Serial.println("[4] 1ドットずつ走査");
  for (int m = 0; m < 2; m++) {
    for (int r = 0; r < 8; r++) {

      Serial.print(m == MATRIX_A ? "  A" : "  B");
      Serial.print(" row ");
      Serial.println(r);

      for (int c = 0; c < 8; c++) {
        leds.clear();
        setDot(m, r, c, true);
        leds.writeDisplay();
        delay(60);
      }
    }
  }

  leds.clear();
  leds.writeDisplay();
  delay(1000);
}
