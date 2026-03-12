namespace advmotctrls {

  struct MotorsPower {
    float pwrLeft;
    float pwrRight;
  };

  struct AccelMotor {
    float pwr;
    bool isDone;
  };

  struct AccelMotors {
    float pwrLeft;
    float pwrRight;
    bool isDoneLeft;
    bool isDoneRight;
  };

  struct MotorPairFloat {
    float left;
    float right;
  };

  struct MotorPairBool {
    bool left;
    bool right;
  };

  int accMotorMinPwr;
  int accMotorMaxPwr;
  int accMotorTotalValue;
  int accMotorAccelValue;
  int accMotorDecelValue;
  bool accMotorIsNeg;

  int accMotorsStartingPwr;
  int accMotorsMaxPwr;
  int accMotorsFinishingPwr;
  int accMotorsTotalValue;
  int accMotorsAccelValue;
  int accMotorsDecelValue;
  bool accMotorsIsNeg;

  MotorPairFloat accMotorsComplexMotionStartingPwrs = { 0, 0 };
  MotorPairFloat accMotorsComplexMotionMaxPwrs = { 0, 0 };
  MotorPairFloat accMotorsComplexMotionFinishingPwrs = { 0, 0 };
  MotorPairFloat accMotorsComplexMotionTotalValue = { 0, 0 };
  MotorPairFloat accMotorsComplexMotionAccelValue = { 0, 0 };
  MotorPairFloat accMotorsComplexMotionDecelValue = { 0, 0 };
  MotorPairBool accMotorsComplexMotionIsNeg = { false, false };

  /**
  * Посчитать ошибку синхронизации моторов шассии с использованием значений с энкодеров и с учётом необходимых скоростей (мощностей) для моторов.
  * Возвращает число ошибки для регулятора.
  * @param eLeft входное значение энкодера левого мотора, eg: 0
  * @param eRight входное значение энкодера правого мотора, eg: 0
  * @param vLeft входное значение скорости (мощности) левого мотора, eg: 50
  * @param vRight входное значение скорости (мощности) правого мотора, eg: 50
  */
  float getErrorSyncMotors(int eLeft, int eRight, int vLeft, int vRight) {
    return (vRight * eLeft) - (vLeft * eRight);
  }

  /**
  * Получить скорости (мощности) для моторов по u воздействию регулятора и с необходимыми скоростями (мощностями).
  * @param u входное значение с регулятора, eg: 0
  * @param vLeft входное значение скорости (мощности) левого мотора, eg: 50
  * @param vRight входное значение скорости (мощности) правого мотора, eg: 50
  */
  MotorsPower getPwrSyncMotors(float u, int vLeft, int vRight) {
    float pLeft = vLeft - (abs(vRight + 1) - abs(vRight)) * u;
    float pRight = vRight + (abs(vLeft + 1) - abs(vLeft)) * u;
    return {
      pLeft,
      pRight
    };
  }

  /**
  * Конфигурация ускорения и замедления шассии двух моторов.
  * @param startPwr входное значение скорости (мощности) на старте, eg: 20
  * @param maxPwr входное значение максимальной скорости (мощности), eg: 50
  * @param endPwr входное значение скорости (мощности) при замедлении, eg: 20
  * @param totalValue значение всей дистанции, eg: 500
  * @param accelValue значение дистанции ускорения, eg: 150
  * @param decelValue значение дистанции замедления, eg: 150
  */
  void accTwoEncLinearMotionConfig(int startPwr, int maxPwr, int endPwr, int totalValue, int accelValue, int decelValue) {
    accMotorsStartingPwr = abs(startPwr);
    accMotorsMaxPwr = abs(maxPwr);
    accMotorsFinishingPwr = abs(endPwr);
    accMotorsTotalValue = abs(totalValue);
    accMotorsAccelValue = abs(accelValue);
    accMotorsDecelValue = abs(decelValue);
    accMotorsIsNeg = maxPwr < 0;
  }

  /**
  * Расчёт ускорения/замедления для двух моторов.
  * @param eLeft входное значение энкодера левого мотора, eg: 0
  * @param eRight входное значение энкодера правого мотора, eg: 0
  */
  AccelMotor accTwoEncLinearMotionCompute(int eLeft, int eRight) {
    bool done = false;
    float pwrOut;
    const float currEnc = (abs(eLeft) + abs(eRight)) / 2;

    if (currEnc >= accMotorsTotalValue) { // Фаза финиша
      pwrOut = 0;
      done = true;
    } else if (currEnc > accMotorsTotalValue - accMotorsDecelValue) { // Фаза замедления
      if (accMotorsDecelValue == 0) pwrOut = accMotorsMaxPwr;
      else pwrOut = (accMotorsMaxPwr - accMotorsFinishingPwr) / pow(accMotorsDecelValue, 2) * pow(currEnc - accMotorsTotalValue, 2) + accMotorsFinishingPwr;
      pwrOut = max(accMotorsFinishingPwr, pwrOut); // Защита от "проседания" ниже EndPwr (мало ли, плавающая точка)
    } else if (currEnc < accMotorsAccelValue) { // Фаза ускорения
      if (accMotorsAccelValue == 0) pwrOut = accMotorsMaxPwr;
      else pwrOut = (accMotorsMaxPwr - accMotorsStartingPwr) / pow(accMotorsAccelValue, 2) * pow(currEnc, 2) + accMotorsStartingPwr;
      pwrOut = max(accMotorsStartingPwr, pwrOut); // Защита от стартовой скорости ниже StartPwr
    } else { // Фаза постоянной скорости (между ускорением и замедлением)
      pwrOut = accMotorsMaxPwr;
    }

    return {
      accMotorsIsNeg ? -pwrOut : pwrOut,
      done
    };
  }

}