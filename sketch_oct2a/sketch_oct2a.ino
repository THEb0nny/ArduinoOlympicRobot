// https://github.com/GyverLibs/uPID
// https://github.com/GyverLibs/GyverMotor

#include "advmotctrls.h"
#include <PinChangeInterrupt.h>
#include <uPID.h>
#include <GyverMotor2.h>

// Настройки пинов
#define MOT_LEFT_ENC_A_PIN 2
#define MOT_LEFT_ENC_B_PIN 3

#define MOT_RIGHT_ENC_A_PIN 8
#define MOT_RIGHT_ENC_B_PIN 9

#define MOT_LEFT_PWR_PIN 6
#define MOT_LEFT_DIR_PIN 7

#define MOT_RIGHT_PWR_PIN 5
#define MOT_RIGHT_DIR_PIN 4

#define MOTOR_GEAR_RATIO 45 // Соотношение редуктора мотора
#define MOTOR_ENCODER_QUAD_MULTIPLIER 4 // Квадратурное умножение
#define MOTOR_ENCODER_PPR 11 // Количество импульсов на одном канале (A или B) за один оборот
#define MOTOR_ENCODER_CPR (MOTOR_ENCODER_PPR * MOTOR_ENCODER_QUAD_MULTIPLIER * MOTOR_GEAR_RATIO) // Итоговое разрешение системы (тиков на оборот выходного вала)

#define WHEEL_DIAMETR 56 // Диаметр колёс в мм
#define BASE_LENGTH 170 // Расстояние между центрами колёс в мм

#define AVG_WINDOW 5 // Количество измерений для усреднения

volatile long encMotorLeftCount = 0; // Cчетчик позиций
volatile long encMotorRightCount = 0;
volatile int8_t motLeftLastEncoded = 0; // Предыдущий код из A/B
volatile int8_t motRightLastEncoded = 0;

GMotor2<DRIVER2WIRE> leftMotor(MOT_LEFT_DIR_PIN, MOT_LEFT_PWR_PIN);
GMotor2<DRIVER2WIRE> rightMotor(MOT_RIGHT_DIR_PIN, MOT_RIGHT_PWR_PIN);

uPID syncChassisPid(P_ERROR | I_SATURATE | D_ERROR | PID_FORWARD);

static long prevEncLeft = 0, prevEncRight = 0;
static long prevEncLeftCount = 0, prevEncRightCount = 0;

// Универсальный обработчик ----------
inline void updateMotorEncoder(const uint8_t pinA, const uint8_t pinB, volatile long* counter, volatile int8_t* lastEncoded) {
  // Считываем состояние каналов
  const int8_t MSB = digitalRead(pinA);
  const int8_t LSB = digitalRead(pinB);

  int8_t encoded = (MSB << 1) | LSB;
  int8_t sum = ((*lastEncoded << 2) | encoded);

  if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000)
    (*counter)++;
  else if (sum == 0b0010 || sum == 0b0100 || sum == 0b1101 || sum == 0b1011)
    (*counter)--;

  *lastEncoded = encoded;
}

// Обёртки для attachInterrupt
void ISR_leftEncoder() {
  updateMotorEncoder(MOT_LEFT_ENC_A_PIN, MOT_LEFT_ENC_B_PIN, &encMotorLeftCount, &motLeftLastEncoded);
}
void ISR_rightEncoder() { 
  updateMotorEncoder(MOT_RIGHT_ENC_A_PIN, MOT_RIGHT_ENC_B_PIN, &encMotorRightCount, &motRightLastEncoded); 
}

void setup() {
  Serial.begin(115200);

  pinMode(MOT_LEFT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(MOT_LEFT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOT_RIGHT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(MOT_RIGHT_ENC_B_PIN, INPUT_PULLUP);

  // Настроить прерывания
  attachInterrupt(digitalPinToInterrupt(MOT_LEFT_ENC_A_PIN), ISR_leftEncoder, CHANGE); // Стандартное прерывание
  attachInterrupt(digitalPinToInterrupt(MOT_LEFT_ENC_B_PIN), ISR_leftEncoder, CHANGE); // Стандартное прерывание
  attachPCINT(digitalPinToPCINT(MOT_RIGHT_ENC_A_PIN), ISR_rightEncoder, CHANGE); // Дополнительное прерывание
  attachPCINT(digitalPinToPCINT(MOT_RIGHT_ENC_B_PIN), ISR_rightEncoder, CHANGE); // Дополнительное прерывание

  leftMotor.setMinDuty(70);
  rightMotor.setMinDuty(70);
  leftMotor.reverse(false);
  rightMotor.reverse(false);
  leftMotor.setDeadtime(5);
  rightMotor.setDeadtime(5);

  syncChassisPid.outMin = -255;
  syncChassisPid.outMax = 255;
  syncChassisPid.setKp(0.005);
  syncChassisPid.setKi(0);
  syncChassisPid.setKd(0);

  solve();
}

void chassisSetPwrCommand(float pLeft, float pRight) {
  leftMotor.setSpeed(pLeft);
  rightMotor.setSpeed(pRight);
}

void chassisBreakStop() {
  leftMotor.brake();
  rightMotor.brake();
}

void chassisFloatStop() {
  leftMotor.stop();
  rightMotor.stop();
}

// Вспомогательная функция расчёта движения на дистанцию в мм
float calculateDistanceToEncRotate(int distance) {
  return (distance / (PI * WHEEL_DIAMETR)) * MOTOR_ENCODER_CPR; // Дистанция в мм, которую нужно пройти
}

// Вспомогательная функция расчёта поворота в градусах
float calculateRotateToEncRotate(float degrees) {
  return ((degrees * BASE_LENGTH) / WHEEL_DIAMETR) * ((float) MOTOR_ENCODER_CPR / 360.0);
}

void linearDistMove(int value, int v) {
  long emlPrev = encMotorLeftCount;
  long emrPrev = encMotorRightCount;

  float motRotateCalc = calculateDistanceToEncRotate(value);

  unsigned long int prevTime = millis();
  while(true) {
    unsigned long currTime = millis();
    float dt = currTime - prevTime;
    prevTime = currTime;

    noInterrupts();
    long eml = encMotorLeftCount - emlPrev;
    long emr = encMotorRightCount - emrPrev;
    interrupts();

    if ((eml + emr) / 2 >= motRotateCalc) break;
    float syncError = advmotctrls::getErrorSyncMotors(currEncLeft, currEncRight, v, v); // Найдите ошибку в управлении двигателей
    syncChassisPid.setDt(dt == 0 ? 1 : dt); // Установить dt регулятору
    float syncU = syncChassisPid.compute(-syncError); // Получить управляющее воздействие от регулятора
    advmotctrls::MotorsPower powers = advmotctrls::getPwrSyncMotors(syncU, v, v); // Узнайте мощность двигателей для регулирования, передав управляющее воздействие
    chassisSetPwrCommand(powers.pwrLeft, powers.pwrRight); // Установить скорости/мощности моторам
    Serial.println(String(currEncLeft) + "\t" + String(currEncRight) + "\t" + String(syncError) + "\t" + String(syncU));
  }
  chassisFloatStop();
}

void solve() {
  linearDistMove(1980, 150);
}

void loop() {
  // static long prevEncLeft = 0, prevEncRight = 0;
  // static long prevEncLeftCount = 0, prevEncRightCount = 0;
  // static long prevTime = 0;

  // Очереди для усреднения
  // static float leftBuf[AVG_WINDOW] = {0}, rightBuf[AVG_WINDOW] = {0};
  // static uint8_t bufIndex = 0;

  // if (currEncLeft != prevEncLeft || currEncRight != prevEncRight) {
  //   Serial.println(String(currEncLeft) + "\t" + String(currEncRight) + "\t" + String(syncError) + "\t" + String(syncU));
  //   prevEncLeft = currEncLeft;
  //   prevEncRight = currEncRight;
  // }

  /*
  // Каждые 100 мс считаем скорость
  if (currTime - prevTime >= 100) {
    noInterrupts();
    long currLeft = encMotorLeftCount;
    long currRight = encMotorRightCount;
    interrupts();

    long dLeft = currLeft - prevEncLeftCount;
    long dRight = currRight - prevEncRightCount;
    prevEncLeftCount = currLeft;
    prevEncRightCount = currRight;

    float dt = (currTime - prevTime) / 1000.0f; // в секундах
    prevTime = currTime;

    float leftRPS = dLeft / (float)TICKS_PER_REV / dt;
    float rightRPS = dRight / (float)TICKS_PER_REV / dt;

    float leftRPM = leftRPS * 60.0f;
    float rightRPM = rightRPS * 60.0f;

    // Добавляем в буфер
    leftBuf[bufIndex] = leftRPM;
    rightBuf[bufIndex] = rightRPM;
    bufIndex = (bufIndex + 1) % AVG_WINDOW;

    // Вычисляем среднее
    float avgLeft = 0, avgRight = 0;
    for (uint8_t i = 0; i < AVG_WINDOW; i++) {
      avgLeft += leftBuf[i];
      avgRight += rightBuf[i];
    }
    avgLeft /= AVG_WINDOW;
    avgRight /= AVG_WINDOW;

    // Serial.println("speed: " + String(avgLeft) + "\t" + String(avgRight));
  }
  */
}