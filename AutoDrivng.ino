static const int THROTTLE_SPEED = 200;
static const int IDEAL_DIST_FROM_WALL   = 150;
static const int WALL_DIST_TOLERANCE   = 30;
static const int STRAFE_SPEED  = 300;
static const int ROTATION_TOLERANCE_DEGREES = 10;
static const int TURN_SPEED = 100;

int throttle = 0;
int strafe = 0;
int turn = 0;
int yawOffset = 0;
void calculateDriveValues(){
  static int targetTurn = 0;
  throttle = THROTTLE_SPEED;
  int distFromRightWallCm = getUltrasonicDistCm(RIGHT_ULTRASONIC);
  distFromRightWallCm = filterBadZeroes(distFromRightWallCm);
  Serial.println(distFromRightWallCm);
  if(distFromRightWallCm > IDEAL_DIST_FROM_WALL + WALL_DIST_TOLERANCE / 2 || distFromRightWallCm == 0){
    //strafe = STRAFE_SPEED;
  }
  else if(distFromRightWallCm < IDEAL_DIST_FROM_WALL - WALL_DIST_TOLERANCE / 2){
    //strafe = -STRAFE_SPEED;
  }
  else{
    strafe = 0;
  }

  //turning
  int rotation = getYaw();
  Serial.println(rotation);
  if(rotation > ROTATION_TOLERANCE_DEGREES + yawOffset + targetTurn){
    turn = TURN_SPEED;
  }
  else if(rotation < -ROTATION_TOLERANCE_DEGREES + yawOffset + targetTurn){
    turn = -TURN_SPEED;
  }
  else{
    turn = 0;
  }
}
int getCalculatedThrottleValue(){
  return throttle;
}
int getCalculatedStrafeValue(){
  return strafe;
}
int getCalculatedTurnValue(){
  return turn;
}
int filterBadZeroes(int dist){
  static int previousDist = -1;
  static int count = 0;
  int tempPrevDist = previousDist;
  previousDist = dist;
  
  if(tempPrevDist == -1){
    tempPrevDist = IDEAL_DIST_FROM_WALL;
  }
  
  if(dist == 0 && count >= 2){
      return 0;
  }
  else if(dist == 0){
    count++;
    return tempPrevDist;
  }
  count = 0;
  return dist;
}

void setGyroYawOffset(){
  yawOffset = getYaw();
}
