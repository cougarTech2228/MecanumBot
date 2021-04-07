#include <Servo.h>
#include <SoftwareSerial.h>

static const int FAILSAFE_LED_PIN = 2;

static const int RIGHT_FRONT_DRIVE_MOTOR_PIN = 9;
static const int LEFT_FRONT_DRIVE_MOTOR_PIN = 10;
static const int RIGHT_REAR_DRIVE_MOTOR_PIN = 11;
static const int LEFT_REAR_DRIVE_MOTOR_PIN = 12;

static const int MIN_PWM_SIGNAL_WIDTH = 1000;
static const int MAX_PWM_SIGNAL_WIDTH = 2000;

static const int SERVO_FULL_REVERSE = 160;
static const int SERVO_STOPPED = 90;
static const int SERVO_FULL_FORWARD = 20;
static const int SERVO_DEADBAND = 8;
static const int SERVO_SAFETY_MARGIN = 20;

static const int TURN_CORRECTION = 10;
static const int TURN_COMPENSATION = 7;

static const int RADIOLINK_TURN_CHANNEL = 2;
static const int RADIOLINK_THROTTLE_CHANNEL = 1;
static const int RADIOLINK_STRAFE_CHANNEL = 4;
static const int RADIOLINK_SHOOT_CANDY_CHANNEL = 6;

static const int RADIOLINK_CONTROLLER_MINIMUM_VALUE = 200;
static const int RADIOLINK_CONTROLLER_NEUTRAL_VALUE = 1000;
static const int RADIOLINK_CONTROLLER_MAXIMUM_VALUE = 1800;

int radioLinkTurnValue = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
int radioLinkThrottleValue = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
int radioLinkStrafeValue = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;

static const int SCALED_DEADBAND = 200; //5% deadband because scaled values are -1000 to 1000

Servo rightFrontDriveMotor, leftFrontDriveMotor, rightRearDriveMotor, leftRearDriveMotor;

bool failSafeEnabled = false;
bool receivedOneSBusPacketSinceReset = false;
static byte sBusBuffer[25];
static int sBusPacketsLost = 0;

// These variables will hold the adjusted forward and reverse speeds
// based on the current Child Mode setting. We've also found that full
// forward and reverse was too fast to control the robot safely so
// we are going to decrease the max range here. The Child Mode setting
// will always less than this initial value.
int adjustedServoFullForward = SERVO_FULL_FORWARD + SERVO_SAFETY_MARGIN;
int adjustedServoFullReverse = SERVO_FULL_REVERSE - SERVO_SAFETY_MARGIN;

/**************************************************************
   setup()
 **************************************************************/
void setup() {

  leftFrontDriveMotor.attach(LEFT_FRONT_DRIVE_MOTOR_PIN,
                             MIN_PWM_SIGNAL_WIDTH, MAX_PWM_SIGNAL_WIDTH);
  rightFrontDriveMotor.attach(RIGHT_FRONT_DRIVE_MOTOR_PIN,
                              MIN_PWM_SIGNAL_WIDTH, MAX_PWM_SIGNAL_WIDTH);
  leftRearDriveMotor.attach(LEFT_REAR_DRIVE_MOTOR_PIN,
                            MIN_PWM_SIGNAL_WIDTH, MAX_PWM_SIGNAL_WIDTH);
  rightRearDriveMotor.attach(RIGHT_REAR_DRIVE_MOTOR_PIN,
                             MIN_PWM_SIGNAL_WIDTH, MAX_PWM_SIGNAL_WIDTH);

  // Set the drive motors to their "stopped" position
  leftFrontDriveMotor.write(SERVO_STOPPED);
  rightFrontDriveMotor.write(SERVO_STOPPED);
  leftRearDriveMotor.write(SERVO_STOPPED);
  rightRearDriveMotor.write(SERVO_STOPPED);

  // put your setup code here, to run once:
  //The SBUS is a non standard baud rate of 100 kbs
  Serial1.begin(100000, SERIAL_8E2);
  Serial1.flush();

  // Setup debug/monitor serial port
  Serial.begin(115200);

  pinMode(FAILSAFE_LED_PIN, OUTPUT);
}

/**************************************************************
   loop()
 **************************************************************/
void loop() {

  if (failSafeEnabled) {
    digitalWrite(FAILSAFE_LED_PIN, HIGH);

    leftFrontDriveMotor.write(SERVO_STOPPED);
    rightFrontDriveMotor.write(SERVO_STOPPED);
    leftRearDriveMotor.write(SERVO_STOPPED);
    rightRearDriveMotor.write(SERVO_STOPPED);
  }
  else {
    digitalWrite(FAILSAFE_LED_PIN, LOW);

    // put your main code here, to run repeatedly:
    static int sBusErrors = 0;
    static int sBusByteIndex = 0;
    byte nextSBusByte;

    //Check the SBus serial port for incoming SBus data
    if (Serial1.available ())
    {
      nextSBusByte = Serial1.read ();
      //This is a new package and it's not the first byte then it's probably the start byte B11110000 (sent MSB)
      //so start reading the 25 byte packet
      if ((sBusByteIndex == 0) && (nextSBusByte != 0x0F))
      {
        // error - keep waiting for the start byte
      }
      else
      {
        sBusBuffer[sBusByteIndex++] = nextSBusByte;  // fill the buffer with the bytes until the end byte B0000000 is received
      }

      // If we've got 25 bytes then this is a good packet so start to decode
      if (sBusByteIndex == 25)
      {
        sBusByteIndex = 0;

        if (sBusBuffer[24] == 0x00)
        {
          processSBusBuffer();
          handleDriveMotors();
        }
        else
        {
          sBusErrors++; //?????
        }
      }
    }
  }
}

/**************************************************************
   processSBusBuffer()
 **************************************************************/
void processSBusBuffer()
{
  // More information on the SBus: https://github.com/uzh-rpg/rpg_quadrotor_control/wiki/SBUS-Protocol
  static int channels[18];

  // 25 byte packet received is little endian. Details of how the package is explained on this website:
  // http://www.robotmaker.eu/ROBOTmaker/quadcopter-3d-proximity-sensing/sbus-graphical-representation

  channels[1]  = ((sBusBuffer[1]       | sBusBuffer[2] << 8)                        & 0x07FF);
  channels[2]  = ((sBusBuffer[2] >> 3  | sBusBuffer[3] << 5)                        & 0x07FF);
  channels[3]  = ((sBusBuffer[3] >> 6  | sBusBuffer[4] << 2 | sBusBuffer[5] << 10)  & 0x07FF);
  channels[4]  = ((sBusBuffer[5] >> 1  | sBusBuffer[6] << 7)                        & 0x07FF);
  channels[5]  = ((sBusBuffer[6] >> 4  | sBusBuffer[7] << 4)                        & 0x07FF);
  channels[6]  = ((sBusBuffer[7] >> 7  | sBusBuffer[8] << 1 | sBusBuffer[9] << 9)   & 0x07FF);
  channels[7]  = ((sBusBuffer[9] >> 2  | sBusBuffer[10] << 6)                       & 0x07FF);
  channels[8]  = ((sBusBuffer[10] >> 5 | sBusBuffer[11] << 3)                       & 0x07FF);
  // We only have 8 channels with our receiver ...
  //  channels[9]  = ((sBusBuffer[12]   | sBusBuffer[13] << 8)                & 0x07FF);
  //  channels[10]  = ((sBusBuffer[13] >> 3 | sBusBuffer[14] << 5)                & 0x07FF);
  //  channels[11] = ((sBusBuffer[14] >> 6 | sBusBuffer[15] << 2 | sBusBuffer[16] << 10) & 0x07FF);
  //  channels[12] = ((sBusBuffer[16] >> 1 | sBusBuffer[17] << 7)                & 0x07FF);
  //  channels[13] = ((sBusBuffer[17] >> 4 | sBusBuffer[18] << 4)                & 0x07FF);
  //  channels[14] = ((sBusBuffer[18] >> 7 | sBusBuffer[19] << 1 | sBusBuffer[20] << 9)  & 0x07FF);
  //  channels[15] = ((sBusBuffer[20] >> 2 | sBusBufferr[21] << 6)                & 0x07FF);
  //  channels[16] = ((sBusBuffer[21] >> 5 | sBusBuffer[22] << 3)                & 0x07FF);
  //  channels[17] = ((sBusBuffer[23])      & 0x0001) ? 2047 : 0;
  //  channels[18] = ((sBusBuffer[23] >> 1) & 0x0001) ? 2047 : 0;

  /*
    Serial.print("CH1: ");
    Serial.println(channels[1]);

    Serial.print("CH2: ");
    Serial.println(channels[2]);

    Serial.print("CH3: ");
    Serial.println(channels[3]);

    Serial.print("CH4: ");
    Serial.println(channels[4]);

    Serial.print("CH5: ");
    Serial.println(channels[5]);
    Serial.print("CH6: ");
    Serial.println(channels[6]);
    Serial.print("CH7: ");
    Serial.println(channels[7]);
    Serial.print("CH8: ");
    Serial.println(channels[8]);

  */
  // Check for signal loss
  if ((sBusBuffer[23] >> 2) & 0x0001)
  {
    sBusPacketsLost++;
    //Serial.print("Signal Lost: ");
    //Serial.println(sBusPacketsLost);

    if (receivedOneSBusPacketSinceReset) {
      failSafeEnabled = true;
    }

    // Make sure the robot doesn't move when the signal is lost
    /*radioLinkTurnValue = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
      radioLinkThrottleValue = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
      radioLinkStrafeValue = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;*/
  }
  else // Everything is okay so process the current values
  {
    receivedOneSBusPacketSinceReset = true;
    //wdt_reset();
    
    radioLinkTurnValue = channels[RADIOLINK_TURN_CHANNEL];
    radioLinkThrottleValue = channels[RADIOLINK_THROTTLE_CHANNEL];
    radioLinkStrafeValue = channels[RADIOLINK_STRAFE_CHANNEL];
  }

  /*
    Serial.print("Throttle: ");
    Serial.print(radioLinkThrottleValue);
    Serial.print(" Turn: ");
    Serial.println(radioLinkTurnValue);
  */

  if (channels[RADIOLINK_SHOOT_CANDY_CHANNEL] == RADIOLINK_CONTROLLER_MAXIMUM_VALUE)
  {
    Serial.println("Shoot Candy");
    failSafeEnabled = true;
    radioLinkTurnValue = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
    radioLinkThrottleValue = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
    radioLinkStrafeValue = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
  }
}

/**************************************************************
   handleDriveMotors()
 **************************************************************/
void handleDriveMotors() {
  int throttle = -map(radioLinkThrottleValue, RADIOLINK_CONTROLLER_MINIMUM_VALUE, RADIOLINK_CONTROLLER_MAXIMUM_VALUE, -1000, 1000);
  int strafe = map(radioLinkStrafeValue, RADIOLINK_CONTROLLER_MINIMUM_VALUE, RADIOLINK_CONTROLLER_MAXIMUM_VALUE, -1000, 1000);
  int turn = map(radioLinkTurnValue, RADIOLINK_CONTROLLER_MINIMUM_VALUE, RADIOLINK_CONTROLLER_MAXIMUM_VALUE, -1000, 1000);

  throttle = applyDeadband(throttle);
  strafe = applyDeadband(strafe);
  turn = applyDeadband(turn);
  /*
    Serial.print("Throttle: ");
    Serial.println(throttle);
    Serial.print("Strafe: ");
    Serial.println(strafe);
    Serial.print("Turn: ");
    Serial.println(turn);
  */

  int frontLeft = throttle + turn - strafe;
  int frontRight = throttle - turn - strafe;
  int backLeft = throttle + turn + strafe;
  int backRight = throttle - turn + strafe;

  /*
    int maximum = calcMax(abs(frontLeft), abs(frontRight), abs(backLeft), abs(backRight));
    Serial.println(maximum);
    if(maximum > 180){
    frontLeft *= 1000;
    frontLeft /= maximum;
    frontRight *= 1000;
    frontRight /= maximum;
    backLeft *= 1000;
    backLeft /= maximum;
    backRight *= 1000;
    backRight /= maximum;

    frontLeft = map(frontLeft, -1000, 1000, 0, 180);
    frontRight = map(frontRight, -1000, 1000, 0, 180);
    backLeft = map(backLeft, -1000, 1000, 0, 180);
    backRight = map(backRight, -1000, 1000, 0, 180);
    }
  */
  /*
    Serial.print("Front Left: ");
    Serial.println(frontLeft);
    Serial.print("Back Left: ");
    Serial.println(backLeft);
    Serial.print("Front Right: ");
    Serial.println(frontRight);
    Serial.print("Back Right: ");
    Serial.println(backRight);
  */
  /*
    int maximum = calcMax(abs(frontLeft), abs(frontRight), abs(backLeft), abs(backRight));
    if(maximum > 1000 || maximum < -1000){
    frontLeft *= 1000;
    frontLeft /= maximum;
    frontRight *= 1000;
    frontRight /= maximum;
    backLeft *= 1000;
    backLeft /= maximum;
    backRight *= 1000;
    backRight /= maximum;
    }
  */
  frontLeft = scale(frontLeft);
  backLeft = scale(backLeft);
  frontRight = scale(frontRight);
  backRight = scale(backRight);

  frontLeft = mapper(frontLeft, -1000, 1000, 0, 180);
  frontRight = mapper(frontRight, -1000, 1000, 0, 180);
  backLeft = mapper(backLeft, -1000, 1000, 0, 180);
  backRight = mapper(backRight, -1000, 1000, 0, 180);

  /*
    Serial.print("Front Left: ");
    Serial.println(frontLeft);
    Serial.print("Back Left: ");
    Serial.println(backLeft);
    Serial.print("Front Right: ");
    Serial.println(frontRight);
    Serial.print("Back Right: ");
    Serial.println(backRight);

  */
  leftFrontDriveMotor.write(frontLeft);
  leftRearDriveMotor.write(backLeft);
  rightFrontDriveMotor.write(frontRight);
  rightRearDriveMotor.write(backRight);
  /*
    bool isDrivingStraight = false;
    int leftThrottle = map(radioLinkThrottleValue, RADIOLINK_CONTROLLER_MINIMUM_VALUE, RADIOLINK_CONTROLLER_MAXIMUM_VALUE, 0, 180);
    int rightThrottle = map(radioLinkThrottleValue, RADIOLINK_CONTROLLER_MINIMUM_VALUE, RADIOLINK_CONTROLLER_MAXIMUM_VALUE, 180, 0);
    Serial.println(leftThrottle);
    Serial.println(rightThrottle);
    if(abs(leftThrottle - SERVO_STOPPED) > SERVO_DEADBAND){
        isDrivingStraight = true;
        leftFrontDriveMotor.write(leftThrottle);
        leftRearDriveMotor.write(leftThrottle);
    }
    else{
    leftFrontDriveMotor.write(SERVO_STOPPED);
    leftRearDriveMotor.write(SERVO_STOPPED);
    }
    if(abs(rightThrottle - SERVO_STOPPED) > SERVO_DEADBAND){
        rightFrontDriveMotor.write(rightThrottle);
        rightRearDriveMotor.write(rightThrottle);
    }
    else{
    rightFrontDriveMotor.write(SERVO_STOPPED);
    rightRearDriveMotor.write(SERVO_STOPPED);
    }
    if(!isDrivingStraight){
    int leftTurn = map(radioLinkTurnValue, RADIOLINK_CONTROLLER_MINIMUM_VALUE, RADIOLINK_CONTROLLER_MAXIMUM_VALUE, 0, 180);
    int rightTurn = map(radioLinkTurnValue, RADIOLINK_CONTROLLER_MINIMUM_VALUE, RADIOLINK_CONTROLLER_MAXIMUM_VALUE, 180, 0);
    if(abs(leftTurn - SERVO_STOPPED) > SERVO_DEADBAND){
        leftFrontDriveMotor.write(leftTurn);
        leftRearDriveMotor.write(-leftTurn);
    }
    else{
    leftFrontDriveMotor.write(SERVO_STOPPED);
    leftRearDriveMotor.write(SERVO_STOPPED);
    }
    if(abs(rightTurn - SERVO_STOPPED) > SERVO_DEADBAND){
        rightFrontDriveMotor.write(-rightTurn);
        rightRearDriveMotor.write(rightTurn);
    }
    else{
    rightFrontDriveMotor.write(SERVO_STOPPED);
    rightRearDriveMotor.write(SERVO_STOPPED);
    }
  */

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
   mapper()
 **************************************************************/
long mapper(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
