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

  enum MotorSide {
    LEFT,
    RIGHT
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

  void zeroMotorProfile(MotorSide side, int absMax) {
    if (absMax == 0) {
      if (side == LEFT) {
        accMotorsComplexMotionStartingPwrs.left = 0;
        accMotorsComplexMotionFinishingPwrs.left = 0;
        accMotorsComplexMotionTotalValue.left = 0;
        accMotorsComplexMotionAccelValue.left = 0;
        accMotorsComplexMotionDecelValue.left = 0;
      } else {
        accMotorsComplexMotionStartingPwrs.right = 0;
        accMotorsComplexMotionFinishingPwrs.right = 0;
        accMotorsComplexMotionTotalValue.right = 0;
        accMotorsComplexMotionAccelValue.right = 0;
        accMotorsComplexMotionDecelValue.right = 0;
      }
    }
  }

  /**
  * Конфигурация ускорения и замедления шасси двух моторов с разными максимальными скоростями.
  * @param startingPwr входное значение скорости (мощности) моторов на старте, eg: 20
  * @param maxPwrLeft входное значение максимальной скорости (мощности) левого мотора, eg: 50
  * @param maxPwrRight входное значение максимальной скорости (мощности) правого мотора, eg: 75
  * @param finishingPwr входное значение скорости (мощности) моторов при замедлении, eg: 20
  * @param totalValue значение всей дистанции, eg: 500
  * @param accelValue значение дистанции ускорения, eg: 100
  * @param decelValue значение дистанции замедления, eg: 150
  */
  void accTwoEncComplexMotionConfig(int startingPwr, int maxPwrLeft, int maxPwrRight, int finishingPwr, int totalValue, int accelValue, int decelValue) {
    int absStartingPwr = abs(startingPwr);
    int absMaxLeft = abs(maxPwrLeft);
    int absMaxRight = abs(maxPwrRight);
    int absFinishingPwr = abs(finishingPwr);

    int absTotalValue = abs(totalValue);
    int absAccelValue = abs(accelValue);
    int absDecelValue = abs(decelValue);

    // Коэффициент пропорциональности (отношение макс. скоростей). Если один из моторов = 0, используем другой как базовый
    float ratio = (absMaxLeft == 0 || absMaxRight == 0) ? 1 : (absMaxLeft < absMaxRight ? (float) absMaxRight / absMaxLeft : (float) absMaxLeft / absMaxRight);

    if (absMaxLeft < absMaxRight && absMaxLeft > 0) { // Левый мотор медленнее - он получает базовые значения
      accMotorsComplexMotionStartingPwrs.left = absStartingPwr;
      accMotorsComplexMotionStartingPwrs.right = absStartingPwr * ratio;
      accMotorsComplexMotionFinishingPwrs.left = absFinishingPwr;
      accMotorsComplexMotionFinishingPwrs.right = absFinishingPwr * ratio;
      accMotorsComplexMotionTotalValue.left = absTotalValue;
      accMotorsComplexMotionTotalValue.right = absTotalValue * ratio;
      accMotorsComplexMotionAccelValue.left = absAccelValue;
      accMotorsComplexMotionAccelValue.right = absAccelValue * ratio;
      accMotorsComplexMotionDecelValue.left = absDecelValue;
      accMotorsComplexMotionDecelValue.right = absDecelValue * ratio;
    } else { // Правый мотор медленнее - он получает базовые значения
      accMotorsComplexMotionStartingPwrs.left = absStartingPwr * ratio;
      accMotorsComplexMotionStartingPwrs.right = absStartingPwr;
      accMotorsComplexMotionFinishingPwrs.left = absFinishingPwr * ratio;
      accMotorsComplexMotionFinishingPwrs.right = absFinishingPwr;
      accMotorsComplexMotionTotalValue.left = absTotalValue * ratio;
      accMotorsComplexMotionTotalValue.right = absTotalValue;
      accMotorsComplexMotionAccelValue.left = absAccelValue * ratio;
      accMotorsComplexMotionAccelValue.right = absAccelValue;
      accMotorsComplexMotionDecelValue.left = absDecelValue * ratio;
      accMotorsComplexMotionDecelValue.right = absDecelValue;
    }
    accMotorsComplexMotionMaxPwrs.left = maxPwrLeft;
    accMotorsComplexMotionMaxPwrs.right = maxPwrRight;

    // КРИТИЧНО для поворота относительно мотора, если скорость мотора 0, обнуляем ВСЕ параметры
    zeroMotorProfile(MotorSide::LEFT, absMaxLeft);
    zeroMotorProfile(MotorSide::RIGHT, absMaxRight);

    accMotorsComplexMotionIsNeg.left = maxPwrLeft < 0;
    accMotorsComplexMotionIsNeg.right = maxPwrRight < 0;
  }

  // Расчёт профиля скорости (мощности) мотора 
  AccelMotor accTwoEncComplexMotionComputeMotorProfile(int enc, int totalValue, int accelValue, int decelValue, int startPwr, int maxPwr, int endPwr, bool isNeg) {
    bool done = false;
    float pwrOut;
    int currEnc = abs(enc);

    int absStartPwr = abs(startPwr);
    int absMaxPwr = abs(maxPwr);
    int absEndPwr = abs(endPwr);

    if (currEnc >= totalValue) { // Фаза финиша
      pwrOut = 0;
      done = true;
    } else if (currEnc > totalValue - decelValue) { // Фаза замедления
      if (decelValue == 0) pwrOut = absMaxPwr;
      else pwrOut = (absMaxPwr - absEndPwr) / pow(decelValue, 2) * pow(currEnc - totalValue, 2) + absEndPwr;
      pwrOut = max((float)absEndPwr, min((float)absMaxPwr, pwrOut));
    } else if (currEnc < accelValue) { // Фаза ускорения
      if (accelValue == 0) pwrOut = absMaxPwr;
      else pwrOut = (absMaxPwr - absStartPwr) / pow(accelValue, 2) * pow(currEnc, 2) + absStartPwr;
      pwrOut = max((float)absStartPwr, min((float)absMaxPwr, pwrOut));
    } else { // Фаза постоянной скорости
      pwrOut = absMaxPwr;
    }

    return {
      isNeg ? -pwrOut : pwrOut,
      done
    };
  }

  /**
  * Расчёт ускорения/замедления для двух моторов с разными максимальными скоростями (мощностями).
  * @param eLeft входное значение энкодера левого мотора, eg: 0
  * @param eRight входное значение энкодера правого мотора, eg: 0
  */
  AccelMotors accTwoEncComplexMotionCompute(int eLeft, int eRight) {
    AccelMotor profileLeft = accTwoEncComplexMotionComputeMotorProfile(
      eLeft,
      accMotorsComplexMotionTotalValue.left, accMotorsComplexMotionAccelValue.left, accMotorsComplexMotionDecelValue.left,
      accMotorsComplexMotionStartingPwrs.left, abs(accMotorsComplexMotionMaxPwrs.left), accMotorsComplexMotionFinishingPwrs.left,
      accMotorsComplexMotionIsNeg.left
    );

    AccelMotor profileRight = accTwoEncComplexMotionComputeMotorProfile(
      eRight,
      accMotorsComplexMotionTotalValue.right, accMotorsComplexMotionAccelValue.right, accMotorsComplexMotionDecelValue.right,
      accMotorsComplexMotionStartingPwrs.right, abs(accMotorsComplexMotionMaxPwrs.right), accMotorsComplexMotionFinishingPwrs.right,
      accMotorsComplexMotionIsNeg.right
    );

    return {
      profileLeft.pwr,
      profileRight.pwr,
      profileLeft.isDone,
      profileRight.isDone
    };
  }

}