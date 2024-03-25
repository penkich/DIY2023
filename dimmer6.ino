/***
 トライアック制御
 非ゼロクロスのＬＥＤフォトトライアックを制御⇒パワートライアックドライブ
 XIAO ESP32S3用
 2023-10-22 by penkich
 2023-11-04 ２系統化とUARTコントロール
 2023-11-06 新しい自作基板用（ioポートが異なる）
 2024-01-13 BH1750（照度センサー） 組み込み dimmer5plus
 2024-01-15 設定操作を付加 dimmer6
 ロータリーエンコーダー
 １回プッシュで(MODE=1)latency テスト、２回目(MODE=2)で照度取得、３回目(MODE=3)で０～５レベルに対応する照度を設定、４回目で運転モード(MODE=0)に戻る
 運転モード(MODE=0)時において照度センサーをスルーするには、TXピンをジャンパーピンでGNDピンに落とす
 参考
 AC-Dimmer-PhaseControl v.1.0 2020.11.03 by meyon
***/ 

#include <RotaryEncoder.h> // https://github.com/mathertel/RotaryEncoder
#include <Wire.h>
#include <BH1750.h> // https://github.com/claws/BH1750
BH1750 bh1750_b;

#include "HT16K33.h" // https://github.com/RobTillaart/HT16K33
// ７セグのドライバーIC(HT16K33）用ライブラリ。回路はadafruitのバックパックに従う。
HT16K33 seg(0x70); // i2cアドレス:0x70, sda:5, scl:6

#include <EEPROM.h>

volatile int interruptCounter1;
volatile int interruptCounter2;
volatile int interruptCounter3;
int zxInterruptCounter1;
int zxInterruptCounter2;
int latency;
bool flag = true;
bool Thru = false; // デバッグ用：運転モード時（MODE=0）センサー制御をスルーする。

hw_timer_t * timer1 = NULL; // トライアック制御
hw_timer_t * timer2 = NULL; // トライアック制御（２系統目）
hw_timer_t * timer3 = NULL; // タイマー３：照度センサー値取得
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

#define PIN_RX 44  // シリアルから制御可能
#define PIN_TX 43
#define PIN_IN1 2  // DT ロータリーエンコーダ
#define PIN_IN2 3  // CLK ロータリーエンコーダ （GPIO3は制限がある?）
#define PIN_SW 1   // ロータリーエンコーダのプッシュスイッチ

RotaryEncoder *encoder = nullptr;
IRAM_ATTR void checkPosition(){
  encoder->tick(); // just call tick() to check the state.
}

int outputPin1 = 8;
int outputPin2 = 9;
int interruptPin1 = 7; // ゼロクロス入力1
int interruptPin2 = 4; // ゼロクロス入力2
int MODE = 0; // MODE=0:運転、1:明るさ可変テスト 2:センサー値表示
bool CLICK = false;


int latencyTable[8] = {
  // レイテンシーテーブル
  // 0:出力100%, 出力8333:0%
  // 0,3328,4160,5005,5830,6656,7500,8333
  0,4160,5005,5830,6656,8333,8333,8333
};

void IRAM_ATTR onTimer1() {
	portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter1++;
  portEXIT_CRITICAL(&timerMux);
}

void IRAM_ATTR onTimer2() {
	portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter2++;
  portEXIT_CRITICAL(&timerMux);
}

void IRAM_ATTR onTimer3() {
	portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter3++;
  portEXIT_CRITICAL(&timerMux);
}

void IRAM_ATTR zeroCross1() {
  zxInterruptCounter1++;
  if (latency == 0){
    digitalWrite(outputPin1, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);

  }else{
    digitalWrite(outputPin1, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    timerStart(timer1);
  }
}

void IRAM_ATTR zeroCross2() {
  zxInterruptCounter2++;
  if (latency == 0){
    digitalWrite(outputPin2, HIGH);
  }else{
    digitalWrite(outputPin2, LOW);
    timerStart(timer2);
  }
}

void IRAM_ATTR SW() {
  if (MODE <3){
    MODE++;
    MODE = MODE % 4;
  }else{
    CLICK = true;
  }
}

int presetlux[6];

int DEFAULT_VAL[6] = {
  100, 200, 500, 1000, 2000, 5000
};
#define DATA_VERSION    "DATA1.1"

struct DATA_SET{
    int val[6];
    char check[10];
};
DATA_SET data;

//データをEEPROMから読み込む。保存データが無い場合デフォルトにする。
void load_data() {
    EEPROM.get<DATA_SET>(0, data);
    if (strcmp(data.check, DATA_VERSION)) { //バージョンをチェック
        //保存データが無い場合デフォルトを設定
        data.val[0] = DEFAULT_VAL[0];
        data.val[1] = DEFAULT_VAL[1];
        data.val[2] = DEFAULT_VAL[2];
        data.val[3] = DEFAULT_VAL[3];
        data.val[4] = DEFAULT_VAL[4];
        data.val[5] = DEFAULT_VAL[5];
    }
}

//EEPROMへの保存
void save_data() {
    //EEPROMに設定を保存する。
    strcpy(data.check, DATA_VERSION);
    EEPROM.put<DATA_SET>(0, data);
    EEPROM.commit(); //大事
}

void setup() {
  Wire.begin();
  Wire.setClock(100000);
  zxInterruptCounter1 = 0;
  zxInterruptCounter2 = 0;
  Serial.begin(9600);                                   //GPIO 43(TX), 44(RX)に入出力されます
  pinMode(PIN_TX, INPUT_PULLUP);
  if (digitalRead(PIN_TX) == 0){  // センサーをスルーする設定は、TXピンをGNDに落として行う（ピン少ないからシャない）。
    Thru = true;
  }else{
    Thru = false;
    Serial1.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);      //Serial1 オブジェクトはピン指定で使用します
  }
  seg.begin();
  seg.displayOn();
  seg.displayClear();
  delay(500);
  seg.displayFloat(8.888); // 7SEG テスト
  delay(500);
  seg.displayFloat(88.88);
  delay(500);
  seg.displayFloat(888.8);
  delay(500);
  seg.displayFloat(8888.0);
  delay(500);
  seg.displayOff();
  seg.displayOn();

  EEPROM.begin(1024); //1kbサイズ
  load_data();
  presetlux[0] = data.val[0];
  presetlux[1] = data.val[1];
  presetlux[2] = data.val[2];
  presetlux[3] = data.val[3];
  presetlux[4] = data.val[4];
  presetlux[5] = data.val[5];

  bh1750_b.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire); // 照度センサー

  pinMode(outputPin1, OUTPUT);
  pinMode(outputPin2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(interruptPin, INPUT_PULLDOWN);
  pinMode(interruptPin1, INPUT);
  pinMode(interruptPin2, INPUT);
	delay(2000);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), zeroCross1, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), zeroCross2, RISING);

  attachInterrupt(digitalPinToInterrupt(PIN_SW), SW, RISING);
	//setTimer();
  timer1 = timerBegin(0, 80, true);
	timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 3800, true);
  timerAlarmEnable(timer1);

  timer2 = timerBegin(1, 80, true);
	timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 3800, true);
  timerAlarmEnable(timer2);

  timer3 = timerBegin(2, 80, true); // 照度センサー値取得タイミング用
	timerAttachInterrupt(timer3, &onTimer3, true);
  timerAlarmWrite(timer3, 1000000, true); // 1sec
  timerAlarmEnable(timer3);

  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE); 
}
void loop() {
  int i = 0;
  int ilevel = 5;
  float light_level_b;
  latency = latencyTable[ilevel];
  seg.displayInt(latency);
  while (true){
    latency = latencyTable[ilevel];
    if (Thru == false){
    while(Serial1.available() > 0){
      int val = Serial1.read();       // 受信したデータを読み込む（外部から制御可能（デバッグ用））
      if(val == '0'){
        ilevel = 0;
      }
      if(val == '1'){
        ilevel = 1;
      }
      if(val == '2'){
        ilevel = 2;
      }
      if(val == '3'){
        ilevel = 3;
      }
      if(val == '4'){
        ilevel = 4;
      }
      if(val == '5'){
        ilevel = 5;
      }
      if(val == '6'){
        ilevel = 6;
      }
      if(val == '7'){
        ilevel = 7;
      }
      if(val == '8'){
        ilevel = 8;
      }
      if(val == '9'){
        ilevel = 9;
      }
      if(val == 'v'){
        String data = Serial1.readStringUntil(';');
        Serial1.println(data);
        latency = data.toInt();
      }
      Serial1.println(ilevel);
    }
    if (Thru == true){
        latency = 0;
    }
    }
    if (latency == 0){
      timerAlarmDisable(timer1);
      timerAlarmDisable(timer2);
      digitalWrite(outputPin1, HIGH);
      digitalWrite(outputPin2, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else if (latency == 8333){
      timerAlarmDisable(timer1);
      timerAlarmDisable(timer2);
      digitalWrite(outputPin1, LOW);
      digitalWrite(outputPin2, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }
    else{
      timerAlarmWrite(timer1, latency, true);
      timerAlarmEnable(timer1);
      timerAlarmWrite(timer2, latency, true);
      timerAlarmEnable(timer2);
    }

    if (interruptCounter1 > 0) {
      portENTER_CRITICAL(&timerMux);
      interruptCounter1--;
      portEXIT_CRITICAL(&timerMux);
      digitalWrite(outputPin1, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      timerStop(timer1);
    }
    if (interruptCounter2 > 0) {
      portENTER_CRITICAL(&timerMux);
      interruptCounter2--;
      portEXIT_CRITICAL(&timerMux);
      digitalWrite(outputPin2, HIGH);
      timerStop(timer2);
    }

    if (interruptCounter3 > 0 && MODE == 2) {
      portENTER_CRITICAL(&timerMux);
      interruptCounter3 = 0;
      portEXIT_CRITICAL(&timerMux);
      if (bh1750_b.measurementReady()) {
        light_level_b = bh1750_b.readLightLevel(); // 照度測定
      }
    }

    if (interruptCounter3 > 10 && MODE == 0) { // 運転モードでは10秒間隔で照度取得
      portENTER_CRITICAL(&timerMux);
      interruptCounter3 = 0;
      portEXIT_CRITICAL(&timerMux);
      if (bh1750_b.measurementReady()) {
        light_level_b = bh1750_b.readLightLevel(); // 照度測定
      }
    }

    encoder->tick();
    int dir = (int)(encoder->getDirection());
    if (dir !=0 && MODE ==1){
      ilevel += dir;
      if (ilevel >= 6){
          ilevel = 6;
      }
      if (ilevel <= 0){
          ilevel = 0;
      }
      latency = latencyTable[ilevel];

      if (latency == 0){
        timerAlarmDisable(timer1);
        timerAlarmDisable(timer2);
        digitalWrite(outputPin1, HIGH);
        digitalWrite(outputPin2, HIGH);
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else if (latency == 8333){
        timerAlarmDisable(timer1);
        timerAlarmDisable(timer2);
        digitalWrite(outputPin1, LOW);
        digitalWrite(outputPin2, LOW);
        digitalWrite(LED_BUILTIN, LOW);
      }
      else{
        timerAlarmWrite(timer1, latency, true);
        timerAlarmEnable(timer1);
        timerAlarmWrite(timer2, latency, true);
        timerAlarmEnable(timer2);
      }
      seg.displayInt(latency);
    }

    if (MODE == 0){ // 運転モード
      if (Thru == true){
        ilevel = 0;
        // 照度センサーの値と、レイテンシーの対応を設定。
      }else{
        if (light_level_b < data.val[0]){ // 照度センサー値がこれより小さいとき、出力１００％
          ilevel = 0;
        } else if(light_level_b < data.val[1]){ //  照度センサー値がこれより小さいとき、出力２番目に大きい。
          ilevel = 1;
        } else if(light_level_b < data.val[2]){
          ilevel = 2;
        } else if(light_level_b < data.val[3]){
          ilevel = 3;
        } else if(light_level_b < data.val[4]){
          ilevel = 4;
        } else if(light_level_b > data.val[5]){ // 照度センサー値がこれより大きいとき、出力０％
          ilevel = 5;
        }
      }
    }
    if (MODE ==0 || MODE ==1){ // 運転モードか手動調整モード 
      seg.displayInt(latency);
    }
    if (MODE == 2){ // 照度表示モード
      ilevel = 9;
      seg.displayFloat(light_level_b);
    }
    if (MODE == 3){ // 設定モード
      ilevel = 7;
      if (flag){
        seg.displayClear();
        delay(500);
        seg.displayInt(i);
        delay(1000);
        flag = false;
      }
      seg.displayInt(data.val[i]);
      if (dir !=0){
        if (data.val[i] >= 1000){
          data.val[i] += dir * 100;
        }else{
          data.val[i] += dir * 10;
        }
        if (data.val[i] >= 9999){
          data.val[i] = 9999;
        }
        if (data.val[i] <= 0){
          data.val[i] = 0;
        }
        seg.displayInt(data.val[i]);
      }
      if (CLICK){
        CLICK = false;
        flag = true;
        i++;
      }
      if (i > 5){
        if (presetlux[0] != data.val[0] || presetlux[1] != data.val[1] || presetlux[2] != data.val[2]
              || presetlux[3] != data.val[3] || presetlux[4] != data.val[4] || presetlux[5] != data.val[5]){
          save_data();
        }
        i = 0;
        MODE = -1;
        seg.displayClear();
        delay(200);
        uint8_t x[4] = {0x73, 0x3e, 0x6d, 0x76}; // "PUSH"
        seg.displayRaw(x);
      }
    }
  }
}
