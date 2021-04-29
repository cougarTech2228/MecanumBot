/**************************************************************
   setupDriveMotors()
 **************************************************************/
void setupDriveMotors(){
  leftFrontDriveMotor.attach(LEFT_FRONT_DRIVE_MOTOR_PIN,
                             MIN_PWM_SIGNAL_WIDTH, MAX_PWM_SIGNAL_WIDTH);
  rightFrontDriveMotor.attach(RIGHT_FRONT_DRIVE_MOTOR_PIN,
                              MIN_PWM_SIGNAL_WIDTH, MAX_PWM_SIGNAL_WIDTH);
  leftRearDriveMotor.attach(LEFT_REAR_DRIVE_MOTOR_PIN,
                            MIN_PWM_SIGNAL_WIDTH, MAX_PWM_SIGNAL_WIDTH);
  rightRearDriveMotor.attach(RIGHT_REAR_DRIVE_MOTOR_PIN,
                             MIN_PWM_SIGNAL_WIDTH, MAX_PWM_SIGNAL_WIDTH);

  // Set the drive motors to their "stopped" position
  stopDriveMotors();

}

/**************************************************************
   handleDriveMotors()
 **************************************************************/
void handleDriveMotors() {
  int throttle = 0;
  int strafe = 0;
  int turn = 0;
  if (!isAuto) {
    throttle = map(radioLinkThrottleValue, RADIOLINK_CONTROLLER_MINIMUM_VALUE, RADIOLINK_CONTROLLER_MAXIMUM_VALUE, -1000, 1000);
    strafe = map(radioLinkStrafeValue, RADIOLINK_CONTROLLER_MINIMUM_VALUE, RADIOLINK_CONTROLLER_MAXIMUM_VALUE, -1000, 1000);
    turn = map(radioLinkTurnValue, RADIOLINK_CONTROLLER_MINIMUM_VALUE, RADIOLINK_CONTROLLER_MAXIMUM_VALUE, -1000, 1000);
  }
  else {
    throttle = 0;
    strafe = 0;
    turn = 0;
  }
  move(throttle, strafe, turn);
}

/**************************************************************
   applyDeadband()
 **************************************************************/
int applyDeadband(int value) {
  if (abs(value) <= SCALED_DEADBAND) {
    return 0;
  }
  else {
    return value;
  }
}

/**************************************************************
   calcMax()
 **************************************************************/
int calcMax(int a, int b, int c, int d) {
  int maximum = max(a, b);
  maximum = max(maximum, c);
  maximum = max(maximum, d);
  return maximum;
}

/**************************************************************
   scale()
 **************************************************************/
int scale(int value) {
  if (value > 1000) {
    return 1000;
  }
  else if (value < -1000) {
    return -1000;
  }
  return value;
}

/**************************************************************
   move()
 **************************************************************/
void move(int throttle, int strafe, int turn) {
  turn = -turn;

  if (!isAuto) {
    throttle = applyDeadband(throttle);
    strafe = applyDeadband(strafe);
    turn = applyDeadband(turn);

    if (childModeEnabled) {
      turn *= CHILD_MODE_TURN_FACTOR;
      strafe *= CHILD_MODE_STRAFE_FACTOR;
      throttle *= CHILD_MODE_THROTTLE_FACTOR;
    }
  }
  int frontLeft = turn + throttle - strafe;
  int frontRight = turn - throttle - strafe;
  int backLeft = turn + throttle + strafe;
  int backRight = turn - throttle + strafe;

  frontLeft = scale(frontLeft);
  backLeft = scale(backLeft);
  frontRight = scale(frontRight);
  backRight = scale(backRight);

  frontLeft = map(frontLeft, -1000, 1000, 0, 180);
  frontRight = map(frontRight, -1000, 1000, 0, 180);
  backLeft = map(backLeft, -1000, 1000, 0, 180);
  backRight = map(backRight, -1000, 1000, 0, 180);

  leftFrontDriveMotor.write(frontLeft);
  leftRearDriveMotor.write(backLeft);
  rightFrontDriveMotor.write(frontRight);
  rightRearDriveMotor.write(backRight);
}

/**************************************************************
   stopDriveMotors()
 **************************************************************/
void stopDriveMotors()
{
  leftFrontDriveMotor.write(SERVO_STOPPED);
  rightFrontDriveMotor.write(SERVO_STOPPED);
  leftRearDriveMotor.write(SERVO_STOPPED);
  rightRearDriveMotor.write(SERVO_STOPPED);
}
