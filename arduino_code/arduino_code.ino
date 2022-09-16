#include <HCSR04.h>

size_t IN1_L = 0, IN2_L = 1;
size_t IN1_R = 2, IN2_R = 3;
size_t LEFT = 0, RIGHT = 1;

#define N_PINS 4
#define N_ENABLE 2
#define FULL_THROTTLE 100
#define NO_THROTTLE 0
#define N_SENSORS 3
enum SensorLocation{SensorLeft=0, SensorCenter=1, SensorRight=2};
enum Control {MoveForward=0, TurnLeft=1, MoveBackward=2, TurnRight=3, Stop=4}; // W,A,S,D

int pins[] = {3, 2, 5, 4};
int pwm_enable[] = {9, 10};

//////////////////////////////
// Ultrasonic sensor code. 
//////////////////////////////

struct HcResult {
  float data[N_SENSORS];
};

#define MAX_DISTANCE 40
HcResult parseData(const HCSR04& sensor) {
  HcResult output;
  for (size_t i = 0; i < N_SENSORS; ++i) {
    output.data[i] = min(
      sensor.dist(i) / MAX_DISTANCE,
      1.0
    );
    if (output.data[i] <= 0.0) {
      output.data[i] = 1.0;
    }
  }
  return output;
}
//////////////////////////////

//////////////////////////////
// Planner virtual code
//////////////////////////////

class BasePlanner {
public:
  virtual Control move(const HcResult& sensorOutput);
};
//////////////////////////////

//////////////////////////////
// Basic planner
//////////////////////////////

class BasicPlanner: public BasePlanner {
  float THRESHOLD = 0.99;
public:
  BasicPlanner() {}

  Control move(const HcResult& sensorOutput) {
    if (sensorOutput.data[SensorLocation::SensorLeft] >= THRESHOLD &&
        sensorOutput.data[SensorLocation::SensorRight] >= THRESHOLD &&
        sensorOutput.data[SensorLocation::SensorCenter] >= THRESHOLD) {
          return Control::MoveForward;
    }
    if (sensorOutput.data[SensorLocation::SensorLeft] < THRESHOLD &&
        sensorOutput.data[SensorLocation::SensorRight] >= THRESHOLD) {
      return Control::TurnRight;          
    }
    if (sensorOutput.data[SensorLocation::SensorRight] < THRESHOLD &&
        sensorOutput.data[SensorLocation::SensorLeft] >= THRESHOLD) {
      return Control::TurnLeft;          
    } 
    return Control::TurnLeft;
  }
  
};
//////////////////////////////


// Codes for directions
int FORWARD[] = {1, 0, 0, 1};
int REVERSE[] = {0, 1, 1, 0};
int STOP[] = {0, 0, 0, 0};
int COUNTERCLOCKWISE[] = {0, 1, 0, 1};
int CLOCKWISE[] = {1, 0, 1, 0};

// Custom code
HCSR04 hc(6, new int[N_SENSORS] {11, 12, 13}, N_SENSORS); //initialisation class HCSR04 (trig pin , echo pin, number of sensor)
BasicPlanner planner;

void setDirectionPins(int pins[N_PINS], int dir[N_PINS]) {
  for (size_t i = 0; i < N_PINS; ++i) {
    digitalWrite(pins[i], dir[i]);
  }
}
void setDirection(int pins[N_PINS], const Control& dir) {
  switch (dir) {
    case Control::MoveForward: {return setDirectionPins(pins, FORWARD);}
    case Control::TurnLeft: {return setDirectionPins(pins, COUNTERCLOCKWISE);}
    case Control::TurnRight: {return setDirectionPins(pins, CLOCKWISE);}
    case Control::MoveBackward: {return setDirectionPins(pins, REVERSE);}
    case Control::Stop: {return setDirectionPins(pins, STOP);}
  }
}

void setEnable(int pwm_enable[N_PINS], int pwm) {
  for (size_t i = 0; i < N_ENABLE; ++i) {
    analogWrite(pwm_enable[i], pwm);
  }
}


void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < N_PINS; ++i) {
    pinMode(pins[i], OUTPUT);
  }
  for (int i = 0; i < N_ENABLE; ++i) {
    pinMode(pwm_enable[i], OUTPUT);
  }
  setDirection(pins, Control::Stop);
  setEnable(pwm_enable, FULL_THROTTLE);
  Serial.begin(9600);
}

void loop() {
  // Turn left every one second
  HcResult sensorOutput = parseData(hc);
  Control movementDirection = planner.move(sensorOutput);
  Serial.println(movementDirection);
  setDirection(pins, movementDirection);
  delay(200);

}
