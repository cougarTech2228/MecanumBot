#include <NewPing.h>

static const int NUM_ULTRASONIC_SENSORS = 4;
static const int ULTRASONIC_MAX_DISTANCE = 200;
static const int ULTRASONIC_PING_INTERVAL = 33;

static const int FRONT_ULTRASONIC_SENSOR_PIN = 4;
static const int RIGHT_ULTRASONIC_SENSOR_PIN = 5;
static const int REAR_ULTRASONIC_SENSOR_PIN = 6;
static const int LEFT_ULTRASONIC_SENSOR_PIN = 7;
static const int ULTRASONIC_SENSOR_TRIGGER_PIN = 3;

NewPing frontSensor(ULTRASONIC_SENSOR_TRIGGER_PIN, FRONT_ULTRASONIC_SENSOR_PIN, ULTRASONIC_MAX_DISTANCE);
NewPing rightSensor(ULTRASONIC_SENSOR_TRIGGER_PIN, RIGHT_ULTRASONIC_SENSOR_PIN, ULTRASONIC_MAX_DISTANCE);
NewPing rearSensor(ULTRASONIC_SENSOR_TRIGGER_PIN, REAR_ULTRASONIC_SENSOR_PIN, ULTRASONIC_MAX_DISTANCE);
NewPing leftSensor(ULTRASONIC_SENSOR_TRIGGER_PIN, LEFT_ULTRASONIC_SENSOR_PIN, ULTRASONIC_MAX_DISTANCE);

NewPing ultrasonicSensorArray[NUM_ULTRASONIC_SENSORS] = { // Sensor object array.
  NewPing(ULTRASONIC_SENSOR_TRIGGER_PIN, FRONT_ULTRASONIC_SENSOR_PIN, ULTRASONIC_MAX_DISTANCE),
  NewPing(ULTRASONIC_SENSOR_TRIGGER_PIN, RIGHT_ULTRASONIC_SENSOR_PIN, ULTRASONIC_MAX_DISTANCE),
  NewPing(ULTRASONIC_SENSOR_TRIGGER_PIN, REAR_ULTRASONIC_SENSOR_PIN, ULTRASONIC_MAX_DISTANCE),
  NewPing(ULTRASONIC_SENSOR_TRIGGER_PIN, LEFT_ULTRASONIC_SENSOR_PIN, ULTRASONIC_MAX_DISTANCE)
};

unsigned long pingTimer[NUM_ULTRASONIC_SENSORS]; // When each pings.
unsigned int cm[NUM_ULTRASONIC_SENSORS]; // Store ping distances.
uint8_t currentSensor = 0; // Which sensor is active.

void ultrasonicSetup() {
	  pingTimer[0] = millis() + 75; // First ping start in ms.
  
  for (uint8_t index = 1; index < NUM_ULTRASONIC_SENSORS; index++) {
    pingTimer[index] = pingTimer[index - 1] + ULTRASONIC_PING_INTERVAL;
  }
}

void ultrasonicLoop() {
	for (uint8_t i = 0; i < NUM_ULTRASONIC_SENSORS; i++) {
		
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += ULTRASONIC_PING_INTERVAL * NUM_ULTRASONIC_SENSORS;
      
      if (i == 0 && currentSensor == NUM_ULTRASONIC_SENSORS - 1){
        oneSensorCycle(); // Do something with results.
      }
      
      ultrasonicSensorArray[currentSensor].timer_stop();
      
      currentSensor = i;
      cm[currentSensor] = 0;
      ultrasonicSensorArray[currentSensor].ping_timer(echoCheck);
    }
  }
}

void echoCheck() { // If ping echo, set distance to array.
  if (ultrasonicSensorArray[currentSensor].check_timer())
    cm[currentSensor] = ultrasonicSensorArray[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Do something with the results.
  for (uint8_t index = 0; index < NUM_ULTRASONIC_SENSORS; index++) {
    /*Serial.print(index);
    Serial.print("=");
    Serial.print(cm[index]);
    Serial.print("cm ");*/


  }
  //Serial.println();
}
int getFrontUltrasonic(int index){
  return cm[index];
}
void ultrasonicDebug(){
  static int index = 0;
    //lcd.setCursor(7, index); // Set the cursor on the third column and first row.
    //lcd.print("   ");
    //lcd.print(cm[index]);
    index++;
    if(index == NUM_ULTRASONIC_SENSORS){
      index = 0;
    }
}
