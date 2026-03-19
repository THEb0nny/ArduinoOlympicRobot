// https://github.com/NicoHood/PinChangeInterrupt
// https://github.com/GyverLibs/uPID
// https://github.com/GyverLibs/GyverMotor2
// https://github.com/GyverLibs/GyverTimers

#include "advmotctrls.h"
#include <PinChangeInterrupt.h>
#include <uPID.h>
#include <GyverMotor2.h>
#include <EncButton.h>
#include <GyverTimers.h>

// Настройки пинов
#define MOT_LEFT_ENC_A_PIN 2
#define MOT_LEFT_ENC_B_PIN 3

#define MOT_RIGHT_ENC_A_PIN 8
#define MOT_RIGHT_ENC_B_PIN 9

#define MOT_LEFT_PWR_PIN 6
#define MOT_LEFT_DIR_PIN 7

#define MOT_RIGHT_PWR_PIN 5
#define MOT_RIGHT_DIR_PIN 4

#define BTN_PIN 11 // Модуль кнопки

#define MOTOR_GEAR_RATIO 45 // Соотношение редуктора мотора
#define MOTOR_ENCODER_MULTIPLIER 4 // Квадратурное умножение
#define MOTOR_ENCODER_PPR 11 // Количество импульсов на одном канале (A или B) за один оборот
#define MOTOR_ENCODER_CPR (MOTOR_ENCODER_PPR * MOTOR_ENCODER_MULTIPLIER * MOTOR_GEAR_RATIO) // Итоговое разрешение системы (тиков на оборот выходного вала)

#define WHEEL_DIAMETR 86.5 // Диаметр колёс в мм
#define BASE_LENGTH 200 // Расстояние между центрами колёс в мм

#define MM_TO_ENC (MOTOR_ENCODER_CPR / (PI * WHEEL_DIAMETR))

#define AVG_WINDOW 5 // Количество измерений для усреднения

enum class MoveUnit {
  Rotations,
  Degrees,
  Seconds,
  MilliSeconds
};

enum class MotionBraking {
  Hold,
  Float,
  Continue
};

volatile long encMotorLeftCount = 0; // Cчетчик позиций
volatile long encMotorRightCount = 0;
volatile int8_t motLeftLastEncoded = 0; // Предыдущий код из A/B
volatile int8_t motRightLastEncoded = 0;

GyverMotor2<GM2::DIR_PWM> leftMotor(MOT_LEFT_DIR_PIN, MOT_LEFT_PWR_PIN);
GyverMotor2<GM2::DIR_PWM> rightMotor(MOT_RIGHT_DIR_PIN, MOT_RIGHT_PWR_PIN);

Button btn(BTN_PIN, INPUT, LOW);

uPID pidChassisSync(P_ERROR | I_SATURATE | D_ERROR | PID_FORWARD);
uPID chassisLeftMotorPid(P_ERROR | I_SATURATE | D_ERROR | PID_FORWARD);
uPID chassisRightMotorPid(P_ERROR | I_SATURATE | D_ERROR | PID_FORWARD);

// static long prevEncLeft = 0, prevEncRight = 0;
// static long prevEncLeftCount = 0, prevEncRightCount = 0;

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

// Универсальный обработчик считываний с энкодеров
inline void updateMotorEncoder(const uint8_t pinA, const uint8_t pinB, volatile long* counter, volatile int8_t* lastEncoded) {
  // Считываем состояние каналов
  const int8_t MSB = digitalRead(pinA);
  const int8_t LSB = digitalRead(pinB);
  int8_t encoded = (MSB << 1) | LSB;
  int8_t sum = ((*lastEncoded << 2) | encoded);
  if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) {
    (*counter)++;
  } else if (sum == 0b0010 || sum == 0b0100 || sum == 0b1101 || sum == 0b1011) {
    (*counter)--;
  }
  *lastEncoded = encoded;
}

// Обёртки для attachInterrupt
void leftEncoderInterrupt() {
  updateMotorEncoder(MOT_LEFT_ENC_A_PIN, MOT_LEFT_ENC_B_PIN, &encMotorLeftCount, &motLeftLastEncoded);
}
void rightEncoderInterrupt() { 
  updateMotorEncoder(MOT_RIGHT_ENC_A_PIN, MOT_RIGHT_ENC_B_PIN, &encMotorRightCount, &motRightLastEncoded); 
}

void chassisSetPwrCommand(float pLeft, float pRight) {
  leftMotor.runSpeed(pLeft);
  rightMotor.runSpeed(pRight);
}

void chassisBreakStop() {
  int dirL = leftMotor.getDir();
  int dirR = rightMotor.getDir();
  if (dirL == 0 && dirR == 0) { // Если оба стоят — ничего не делаем
    chassisFloatStop();
    return;
  }
  chassisSetPwrCommand(-dirL * 255, -dirR * 255);
  delay(10);
  chassisFloatStop();
}

void chassisHoldStop(int holdTimeMs) {
  unsigned long holdTimeUs = holdTimeMs * 1000UL;
  long emlPrev = encMotorLeftCount;
  long emrPrev = encMotorRightCount;
  chassisLeftMotorPid.setKp(1);
  // chassisLeftMotorPid.setKd(0);
  chassisLeftMotorPid.setpoint = 0;
  chassisRightMotorPid.setKp(1);
  // chassisRightMotorPid.setKd(0);
  chassisRightMotorPid.setpoint = 0;
  unsigned long int startTime = micros();
  unsigned long int prevTime = micros();
  while(micros() - startTime <= holdTimeUs) {
    unsigned long currTime = micros();
    float dt = (currTime - prevTime) / 1000.0;
    prevTime = currTime;
    noInterrupts();
    long eml = encMotorLeftCount - emlPrev;
    long emr = encMotorRightCount - emrPrev;
    interrupts();
    chassisLeftMotorPid.setDt(dt == 0 ? 1 : dt); // Установить dt регулятору
    chassisRightMotorPid.setDt(dt == 0 ? 1 : dt); // Установить dt регулятору
    float uLeft = chassisLeftMotorPid.compute(eml); // Получить управляющее воздействие от регулятора
    float uRight = chassisRightMotorPid.compute(emr); // Получить управляющее воздействие от регулятора
    chassisSetPwrCommand(uLeft, uRight); // Установить скорости/мощности моторам
    // Serial.println(String(eml) + "\t" + String(emr) + "\t" + String(uLeft) + "\t" + String(uRight));
    pauseUntilTimeUs(currTime, 1000);
  }
  chassisFloatStop();
}

void chassisFloatStop() {
  leftMotor.stop();
  rightMotor.stop();
}

void pauseUntilTimeUs(unsigned long startTimeUs, unsigned long delayUs) {
  if (startTimeUs == 0) startTimeUs = micros();
  while (micros() - startTimeUs < delayUs) delayMicroseconds(50);
}

// Вспомогательная функция расчёта движения на дистанцию в мм
float distanceToTicks(int distance) {
  return distance * MM_TO_ENC; // Дистанция в мм, которую нужно пройти
}

// Вспомогательная функция расчёта поворота в градусах
float turnToTicks(float degrees) {
  return (PI * BASE_LENGTH * degrees / 360.0) * MM_TO_ENC;
}

void syncMovement(int vLeft, int vRight, float value, MoveUnit unit, MotionBraking braking) {
  if ((vLeft == 0 && vRight == 0) ||
    ((unit == MoveUnit::Rotations || unit == MoveUnit::Degrees) && value == 0) ||
    ((unit == MoveUnit::Seconds || unit == MoveUnit::MilliSeconds) && value <= 0)) {
    chassisHoldStop(50);
    return;
  }
  vLeft = constrain((int) vLeft, -255, 255); // Ограничиваем скорость левого мотора от -255 до 255 и отсекаем дробную часть
  vRight = constrain((int) vRight, -255, 255); // Ограничиваем скорость правого мотора от -255 до 255 и отсекаем дробную часть
  long emlPrev = encMotorLeftCount;
  long emrPrev = encMotorRightCount;
  long targetAngle = 0;
  unsigned long targetTimeUs = 0;
  switch (unit) {
    case MoveUnit::Rotations:
      targetAngle = value * 360;
      break;
    case MoveUnit::Degrees:
      targetAngle = value;
      break;
    case MoveUnit::Seconds:
      targetTimeUs = value * 1000000UL;
      break;
    case MoveUnit::MilliSeconds:
      targetTimeUs = value * 1000UL;
      break;
    default:
      return;
  }
  long emlTarget = vLeft != 0 ? targetAngle : 0;
  long emrTarget = vRight != 0 ? targetAngle : 0;
  unsigned long prevTime = micros();
  unsigned long startTime = prevTime;
  while (true) {
    unsigned long currTime = micros();
    float dt = (currTime - prevTime) / 1000.0f; // Мсек
    prevTime = currTime;
    // Условие по времени
    if (targetTimeUs > 0 && (currTime - startTime) >= targetTimeUs) break;
    long eml = encMotorLeftCount - emlPrev;
    long emr = encMotorRightCount - emrPrev;
    // Условие по углу
    if (targetAngle > 0 && abs(eml) >= abs(emlTarget) && abs(emr) >= abs(emrTarget)) break;
    float error = advmotctrls::getErrorSyncMotors(eml, emr, vLeft, vRight);
    pidChassisSync.setDt(dt == 0 ? 1 : dt); // Установить dt регулятору
    float u = pidChassisSync.compute(-error);
    advmotctrls::MotorsPower powers = advmotctrls::getPwrSyncMotors(u, vLeft, vRight);
    chassisSetPwrCommand(powers.pwrLeft, powers.pwrRight);
    pauseUntilTimeUs(currTime, 1000);
  }

  if (braking == MotionBraking::Hold) {
    chassisHoldStop(100);
  } else if (braking == MotionBraking::Float) {
    chassisFloatStop();
  } else if (braking == MotionBraking::Continue) {
    chassisSetPwrCommand(vLeft, vRight);
  }
}

void linearDistMove(float dist, int v, MotionBraking braking) {
  if (v == 0 || dist == 0) {
    chassisHoldStop(50);
    return;
  }
  if (v < 0) Serial.println("Warning: v is negative (" + String(v) + "). Using absolute value.");
  v = abs(v); // Модуль скорости
  int dirSign = (dist > 0) - (dist < 0);
  long mRotCalc = round(distanceToTicks(abs(dist)));
  syncMovement(v * dirSign, v * dirSign, mRotCalc, MoveUnit::Degrees, braking);
}

void distMove(float dist, int vLeft, int vRight, MotionBraking braking) {
  if (dist == 0 || vLeft == 0 || vRight == 0) {
    chassisHoldStop(50);
    return;
  }
  if (vLeft < 0) Serial.println("Warning: vLeft is negative (" + String(vLeft) + "). Using absolute value.");
  if (vRight < 0) Serial.println("Warning: vRight is negative (" + String(vRight) + "). Using absolute value.");
  // Берём модуль
  vLeft = abs(vLeft);
  vRight = abs(vRight);
  int dirSign = (dist > 0) - (dist < 0);
  long mRotCalc = (long)round(distanceToTicks(abs(dist)));
  syncMovement(vLeft * dirSign, vRight * dirSign, mRotCalc, MoveUnit::Degrees, braking);
}

void spinTurn(int deg, int v) {
  if (deg == 0 || v == 0) {
    chassisHoldStop(50);
    return;
  }
  long emlPrev = encMotorLeftCount;
  long emrPrev = encMotorRightCount;
  v = constrain(abs(v), 0, 255);
  float calcMotRot = round(turnToTicks(abs(deg)));
  const int vLeft = deg < 0 ? -v : v;
  const int vRight = deg > 0 ? -v : v;
  // unsigned long int startTime = micros();
  unsigned long int prevTime = micros();
  while(true) {
    unsigned long currTime = micros();
    float dt = (currTime - prevTime) / 1000.0;
    prevTime = currTime;
    noInterrupts();
    long eml = encMotorLeftCount - emlPrev;
    long emr = encMotorRightCount - emrPrev;
    interrupts();
    if ((abs(eml) + abs(emr)) / 2 >= calcMotRot) break;
    float syncError = advmotctrls::getErrorSyncMotors(eml, emr, vLeft, vRight); // Найдите ошибку в управлении двигателей
    pidChassisSync.setDt(dt == 0 ? 1 : dt); // Установить dt регулятору
    float syncU = pidChassisSync.compute(-syncError); // Получить управляющее воздействие от регулятора
    advmotctrls::MotorsPower powers = advmotctrls::getPwrSyncMotors(syncU, vLeft, vRight); // Узнайте мощность двигателей для регулирования, передав управляющее воздействие
    chassisSetPwrCommand(powers.pwrLeft, powers.pwrRight); // Установить скорости/мощности моторам
    // Serial.println(String(eml) + "\t" + String(emr) + "\t" + String(syncError) + "\t" + String(syncU));
    pauseUntilTimeUs(currTime, 1000);
  }
  // int dir = (v > 0) ? 1 : -1;
  // chassisBreakStop(dir);
  chassisHoldStop(100);
}

// Float map
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Нормализация значения отражения в диапазон
float normalizingReflectionRawValue(int refRawVal, int bRefRawVal, int wRefRawVal) {
  float refVal = floatMap(refRawVal, bRefRawVal, wRefRawVal, 0.0, 255.0);
  refVal = constrain(refVal, 0.0, 255.0);
  return refVal;
}

// Прерывание таймера 2 на канал А (3 и 11 пины)
ISR(TIMER2_A) {
  btn.tick();
}

void setup() {
  Serial.begin(115200);

  pinMode(MOT_LEFT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(MOT_LEFT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(MOT_RIGHT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(MOT_RIGHT_ENC_B_PIN, INPUT_PULLUP);

  // Настроить прерывания
  attachInterrupt(digitalPinToInterrupt(MOT_LEFT_ENC_A_PIN), leftEncoderInterrupt, CHANGE); // Стандартное прерывание на левый энкодер
  attachInterrupt(digitalPinToInterrupt(MOT_LEFT_ENC_B_PIN), leftEncoderInterrupt, CHANGE); // Стандартное прерывание на левый энкодер
  attachPCINT(digitalPinToPCINT(MOT_RIGHT_ENC_A_PIN), rightEncoderInterrupt, CHANGE); // Дополнительное прерывание на правый энкодер
  attachPCINT(digitalPinToPCINT(MOT_RIGHT_ENC_B_PIN), rightEncoderInterrupt, CHANGE); // Дополнительное прерывание на правый энкодер

  Timer2.setPeriod(1000);
  Timer2.enableISR(CHANNEL_A);

  leftMotor.setMinDuty(70);
  rightMotor.setMinDuty(70);
  leftMotor.setReverse(false);
  rightMotor.setReverse(false);
  leftMotor.setDeadtime(100);
  rightMotor.setDeadtime(100);

  pidChassisSync.outMin = -255;
  pidChassisSync.outMax = 255;
  pidChassisSync.setKp(0.005);
  pidChassisSync.setKi(0);
  pidChassisSync.setKd(0.05);
  
  chassisLeftMotorPid.outMin = -255;
  chassisLeftMotorPid.outMax = 255;
  chassisRightMotorPid.outMin = -255;
  chassisRightMotorPid.outMax = 255;

  solve();
}

void solve() {
  Serial.println("Press to start");
  while (!btn.pressing()) delay(1);
  Serial.println("Start!");
  // linearDistMove(100, 255, MotionBraking::Hold);
  // delay(1000);
  // linearDistMove(-100, 255, MotionBraking::Hold);
  // spinTurn(-90, 255);
  // delay(1000);
  // spinTurn(90, 255);
  distMove(500, 100, 255, MotionBraking::Hold);
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