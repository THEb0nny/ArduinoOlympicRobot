namespace advmotctrls {

  struct MotorsPower {
    float pwrLeft;
    float pwrRight;
  };

  float getErrorSyncMotors(int eLeft, int eRight, int vLeft, int vRight) {
    return (vRight * eLeft) - (vLeft * eRight);
  }

  MotorsPower getPwrSyncMotors(float u, int vLeft, int vRight) {
    float pLeft = vLeft - (abs(vRight + 1) - abs(vRight)) * u;
    float pRight = vRight + (abs(vLeft + 1) - abs(vLeft)) * u;
    return {pLeft, pRight};
  }

}