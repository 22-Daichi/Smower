#include <Arduino.h>

volatile unsigned long lastInterruptTime = 0;
volatile unsigned long currentTime = 0;

const int inputPin = 14;  // 入力ピン（プルアップ）
const int outputPin = 12; // 出力ピン

volatile bool triggered = false;

void IRAM_ATTR handleInterrupt()
{
  currentTime = millis();
  if (currentTime - lastInterruptTime > 50)
  { // 50ms以上の間隔のみ有効
    triggered = true;
    digitalWrite(outputPin, LOW);
    lastInterruptTime = currentTime;
  }
}
void setup()
{
  Serial.begin(115200);

  pinMode(inputPin, INPUT_PULLDOWN); // プルアップ入力
  pinMode(outputPin, OUTPUT);      // 出力モード
  digitalWrite(outputPin, HIGH);   // 初期はHIGH

  // 割り込みを設定（FALLING: HIGH→LOWのエッジで発火）
  attachInterrupt(digitalPinToInterrupt(inputPin), handleInterrupt, RISING);
}

void loop()
{
  if (triggered && digitalRead(inputPin) == HIGH)// スイッチ押されてない
  {
    triggered = false;
    digitalWrite(outputPin, HIGH);
  }

  // 必要に応じてdelayや他の処理
  delay(10);
}