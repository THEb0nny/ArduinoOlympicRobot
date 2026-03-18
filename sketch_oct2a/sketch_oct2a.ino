// https://github.com/GyverLibs/uPID
// https://github.com/GyverLibs/GyverMotor2
// https://github.com/NicoHood/PinChangeInterrupt
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

#define BTN_PIN 11

#define MOTOR_GEAR_RATIO 45 // Соотношение редуктора мотора
#define MOTOR_ENCODER_MULTIPLIER 4 // Квадратурное умножение
#define MOTOR_ENCODER_PPR 11 // Количество импульсов на одном канале (A или B) за один оборот
#define MOTOR_ENCODER_CPR (MOTOR_ENCODER_PPR * MOTOR_ENCODER_MULTIPLIER * MOTOR_GEAR_RATIO) // Итоговое разрешение системы (тиков на оборот выходного вала)

#define WHEEL_DIAMETR 86.5 // Диаметр колёс в мм
#define BASE_LENGTH 200 // Расстояние между центрами колёс в мм

#define MM_TO_ENC (MOTOR_ENCODER_CPR / (PI * WHEEL_DIAMETR))

#define AVG_WINDOW 5 // Количество измерений для усреднения

volatile long encMotorLeftCount = 0; // Cчетчик позиций
volatile long encMotorRightCount = 0;
volatile int8_t motLeftLastEncoded = 0; // Предыдущий код из A/B
volatile int8_t motRightLastEncoded = 0;

GyverMotor2<GM2::DIR_PWM> leftMotor(MOT_LEFT_DIR_PIN, MOT_LEFT_PWR_PIN);
GyverMotor2<GM2::DIR_PWM> rightMotor(MOT_RIGHT_DIR_PIN, MOT_RIGHT_PWR_PIN);

Button btn(BTN_PIN, INPUT, LOW);

uPID syncChassisPid(P_ERROR | I_SATURATE | D_ERROR | PID_FORWARD);
uPID holdStopLeftMotorPid(P_ERROR | I_SATURATE | D_ERROR | PID_FORWARD);
uPID holdStopRightMotorPid(P_ERROR | I_SATURATE | D_ERROR | PID_FORWARD);

static long prevEncLeft = 0, prevEncRight = 0;
static long prevEncLeftCount = 0, prevEncRightCount = 0;

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
  leftMotor.setDeadtime(5);
  rightMotor.setDeadtime(5);

  syncChassisPid.outMin = -255;
  syncChassisPid.outMax = 255;
  syncChassisPid.setKp(0.005);
  syncChassisPid.setKi(0);
  syncChassisPid.setKd(0.5);

  holdStopLeftMotorPid.outMin = -255;
  holdStopLeftMotorPid.outMax = 255;
  holdStopLeftMotorPid.setKp(1);
  holdStopLeftMotorPid.setKd(0);
  holdStopLeftMotorPid.setpoint = 0;

  holdStopRightMotorPid.outMin = -255;
  holdStopRightMotorPid.outMax = 255;
  holdStopRightMotorPid.setKp(1);
  holdStopRightMotorPid.setKd(0);
  holdStopRightMotorPid.setpoint = 0;

  solve();
}

void chassisSetPwrCommand(float pLeft, float pRight) {
  leftMotor.runSpeed(pLeft);
  rightMotor.runSpeed(pRight);
}

void chassisBreakStop(int dir) {
  chassisSetPwrCommand(-dir * 255, -dir * 255);
  delay(10);
  chassisFloatStop();
}

void chassisHoldStop(int brakeTime) {
  long emlPrev = encMotorLeftCount;
  long emrPrev = encMotorRightCount;

  unsigned long int startTime = millis();
  unsigned long int prevTime = millis();
  while(millis() - startTime <= brakeTime) {
    unsigned long currTime = millis();
    float dt = currTime - prevTime;
    prevTime = currTime;
    noInterrupts();
    long eml = encMotorLeftCount - emlPrev;
    long emr = encMotorRightCount - emrPrev;
    interrupts();
    holdStopLeftMotorPid.setDt(dt == 0 ? 1 : dt); // Установить dt регулятору
    holdStopRightMotorPid.setDt(dt == 0 ? 1 : dt); // Установить dt регулятору
    float leftU = holdStopLeftMotorPid.compute(eml); // Получить управляющее воздействие от регулятора
    float rightU = holdStopRightMotorPid.compute(emr); // Получить управляющее воздействие от регулятора
    chassisSetPwrCommand(leftU, rightU); // Установить скорости/мощности моторам
    // Serial.println(String(eml) + "\t" + String(emr) + "\t" + String(leftU) + "\t" + String(rightU));
    pauseUntilTime(currTime, 1);
  }
  chassisFloatStop();
}

void chassisFloatStop() {
  leftMotor.stop();
  rightMotor.stop();
}

void pauseUntilTime(unsigned long startTime, unsigned long delay) {
  if (startTime == 0) startTime = millis();
  unsigned long endTime = startTime + delay;
  while (millis() - startTime < delay) delayMicroseconds(100);
}

// Вспомогательная функция расчёта движения на дистанцию в мм
float distanceToTicks(int distance) {
  return distance * MM_TO_ENC; // Дистанция в мм, которую нужно пройти
}

// Вспомогательная функция расчёта поворота в градусах
float turnToTicks(float degrees) {
  return (PI * BASE_LENGTH * degrees / 360.0) * MM_TO_ENC;
}

void linearDistMove(int value, int v) {
  long emlPrev = encMotorLeftCount;
  long emrPrev = encMotorRightCount;

  float motRotateCalc = round(distanceToTicks(value));

  unsigned long int prevTime = millis();
  while(true) {
    unsigned long currTime = millis();
    float dt = currTime - prevTime;
    prevTime = currTime;

    noInterrupts();
    long eml = encMotorLeftCount - emlPrev;
    long emr = encMotorRightCount - emrPrev;
    interrupts();

    if ((abs(eml) + abs(emr)) / 2 >= motRotateCalc) break;
    float syncError = advmotctrls::getErrorSyncMotors(eml, emr, v, v); // Найдите ошибку в управлении двигателей
    syncChassisPid.setDt(dt == 0 ? 1 : dt); // Установить dt регулятору
    float syncU = syncChassisPid.compute(-syncError); // Получить управляющее воздействие от регулятора
    advmotctrls::MotorsPower powers = advmotctrls::getPwrSyncMotors(syncU, v, v); // Узнайте мощность двигателей для регулирования, передав управляющее воздействие
    chassisSetPwrCommand(powers.pwrLeft, powers.pwrRight); // Установить скорости/мощности моторам
    // Serial.println(String(eml) + "\t" + String(emr) + "\t" + String(syncError) + "\t" + String(syncU) + "\t" + String(powers.pwrLeft) + "\t" + String(powers.pwrRight));
    pauseUntilTime(currTime, 1);
  }
  // int dir = (v > 0) ? 1 : -1;
  // chassisBreakStop(dir);
  chassisHoldStop(50);
}

void spinTurn(int deg, int v) {
  if (deg == 0 || v == 0) {
    chassisFloatStop();
    return;
  }

  long emlPrev = encMotorLeftCount;
  long emrPrev = encMotorRightCount;

  v = constrain(abs(v), 0, 255);

  float calcMotRot = round(turnToTicks(deg));

  const int vLeft = deg < 0 ? -v : v;
  const int vRight = deg > 0 ? -v : v;

  unsigned long int prevTime = millis();
  // unsigned long int startTime = millis();
  while(true) {
    unsigned long currTime = millis();
    float dt = currTime - prevTime;
    prevTime = currTime;

    noInterrupts();
    long eml = encMotorLeftCount - emlPrev;
    long emr = encMotorRightCount - emrPrev;
    interrupts();

    if ((abs(eml) + abs(emr)) / 2 >= abs(calcMotRot)) break;
    float syncError = advmotctrls::getErrorSyncMotors(eml, emr, vLeft, vRight); // Найдите ошибку в управлении двигателей
    syncChassisPid.setDt(dt == 0 ? 1 : dt); // Установить dt регулятору
    float syncU = syncChassisPid.compute(-syncError); // Получить управляющее воздействие от регулятора
    advmotctrls::MotorsPower powers = advmotctrls::getPwrSyncMotors(syncU, vLeft, vRight); // Узнайте мощность двигателей для регулирования, передав управляющее воздействие
    chassisSetPwrCommand(powers.pwrLeft, powers.pwrRight); // Установить скорости/мощности моторам
    // Serial.println(String(eml) + "\t" + String(emr) + "\t" + String(syncError) + "\t" + String(syncU));
    pauseUntilTime(currTime, 1);
  }
  // int dir = (v > 0) ? 1 : -1;
  // chassisBreakStop(dir);
  chassisHoldStop(5000);
}

void solve() {
  Serial.println("Press to start");
  while (!btn.pressing()) delay(1);
  Serial.println("Start!");
  // linearDistMove(100, 150);
  // delay(1000);
  // linearDistMove(100, -150);
  spinTurn(-90, 255);
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