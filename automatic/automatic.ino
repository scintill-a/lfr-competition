#include <Arduino.h>
#include <QTRSensors.h>
#include <EEPROM.h>

#define SENSOR_CNT 5
#define ADDR_BASE 10
#define CENTER_VAL 2000
#define OUT_OF_LINE_ERROR_VALUE 20

const int PWM1 = 6;
const int dir1 = 2;
const int PWM2 = 5;
const int dir2 = 3;

const uint8_t STATUS_LED = 13;
const uint8_t MODE_LED = 12;

#define BLACK_LINE_WHITE_TRACK 1
#define WHITE_LINE_BLACK_TRACK 0

int BaseSpeed = 70;
int MaxSpeed = 250;

#define DEFAULT_KP 0.05
#define DEFAULT_KD 0.4
#define DEFAULT_KI 0.0
struct TuningConfig {
  float Kp;
  float Ki;
  float Kd;
  int baseSpeed;
  int maxSpeed;
};

TuningConfig conservative = { 0.05f, 0.0f, 0.4f, 50, 90 };
TuningConfig balanced =     { 0.05f, 0.0f, 0.4f, 70, 120 };
TuningConfig aggressive =   { 0.05f,  0.0f, 0.4f, 100, 150 };

TuningConfig configs[] = { conservative, balanced, aggressive };
int aggressionLevel = 0; 

#define TURN_SPEED_REDUCTION_ENABLED 1
#define TURN_SPEED_REDUCTION_PERCENT 10

QTRSensors qtr;
const uint8_t SensorCount = SENSOR_CNT;
unsigned int sensorValues[SensorCount];
int sensAvg[SENSOR_CNT];
bool isDebug = true;

uint8_t trackType = BLACK_LINE_WHITE_TRACK;

int error = 0;
int error_dir = 0;
int P = 0;
int I = 0;
int D = 0;
int previousError = 0;
int PID_value = 0;
float Kp = DEFAULT_KP;
float Ki = DEFAULT_KI;
float Kd = DEFAULT_KD;

int leftMotorOffset = 0;
int rightMotorOffset = 0;

int loopDelay = 0;

int button1 = 1;
int button2 = 1;
bool isCalibrated = false;
bool isRunning = false;

#define MID_3_SENSORS_HIGH (s1 == 1 && s2 == 1 && s3 == 1)
#define MID_3_SENSORS_LOW (s1 == 0 && s2 == 0 && s3 == 0)

void init_motor();
void motordrive(const char *directionL, int powerL, const char *directionR, int powerR);
void motorsteer(const char *robotdirection, int power);
void saveValue(int addr, uint16_t value);
uint16_t readValue(int addr);
void restoreSensorValue();
void storeSensorValue();
void initialize(bool debug);
void readButton();
int getButtonEvt(int idx);
void performCalibration();
void readSensors();
void calculatePID();
void controlMotors();
void shortBrake(int duration);
void turnCW(int leftSpeed, int rightSpeed);
void turnCCW(int leftSpeed, int rightSpeed);
void moveStraight(int leftSpeed, int rightSpeed);
void stopMotors();
bool isOutOfLine(unsigned int* sensors);
uint16_t getSensorReadings();
int getCalculatedError(int mode);
void indicateOn();
void indicateOff();
void indicateInversionOn();
void indicateInversionOff();
void waitForStart();

void init_motor();
void motordrive(const char *directionL, int powerL, const char *directionR, int powerR);
void motorsteer(const char *robotdirection, int power);
void saveValue(int addr, uint16_t value);
uint16_t readValue(int addr);
void restoreSensorValue();
void storeSensorValue();
void initialize(bool debug);
void readButton();
int getButtonEvt(int idx);
void performCalibration();
void readSensors();
void calculatePID();
void controlMotors();
void shortBrake(int duration);
void turnCW(int leftSpeed, int rightSpeed);
void turnCCW(int leftSpeed, int rightSpeed);
void moveStraight(int leftSpeed, int rightSpeed);
void stopMotors();
bool isOutOfLine(unsigned int* sensors);
uint16_t getSensorReadings();
int getCalculatedError(int mode);
void indicateInversionOn();
void indicateInversionOff();


void indicateInversionOn() {
  static bool lastState = false;
  if (!lastState) {
    Serial.println(">>> TRACK INVERTED: WHITE LINE ON BLACK <<<");
    lastState = true;
  }
}

void indicateInversionOff() {
  static bool lastState = true;
  if (lastState) {
    lastState = false;
  }
}

void init_motor() {
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
}

void motordrive(const char *directionL, int powerL, const char *directionR, int powerR) {
  if (strcmp(directionL, "forward") == 0) {
    digitalWrite(dir1, HIGH);
  } else if (strcmp(directionL, "backward") == 0) {
    digitalWrite(dir1, LOW);
  }

  if (strcmp(directionR, "forward") == 0) {
    digitalWrite(dir2, LOW);
  } else if (strcmp(directionR, "backward") == 0) {
    digitalWrite(dir2, HIGH);
  }

  analogWrite(PWM1, powerL);
  analogWrite(PWM2, powerR);
}

void motorsteer(const char *robotdirection, int power) {
  if (strcmp(robotdirection, "forward") == 0) {
    digitalWrite(dir1, HIGH);
    analogWrite(PWM1, power);
    digitalWrite(dir2, LOW);
    analogWrite(PWM2, power);
  } else if (strcmp(robotdirection, "stop") == 0) {
    digitalWrite(dir1, HIGH);
    analogWrite(PWM1, 0);
    digitalWrite(dir2, HIGH);
    analogWrite(PWM2, 0);
  }
}

void moveStraight(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  motordrive("forward", leftSpeed, "forward", rightSpeed);
}

void turnCW(int leftSpeed, int rightSpeed) {
#if TURN_SPEED_REDUCTION_ENABLED == 1
  leftSpeed = (leftSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
  rightSpeed = (rightSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  motordrive("forward", leftSpeed, "backward", rightSpeed);
}

void turnCCW(int leftSpeed, int rightSpeed) {
#if TURN_SPEED_REDUCTION_ENABLED == 1
  leftSpeed = (leftSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
  rightSpeed = (rightSpeed * (100 - TURN_SPEED_REDUCTION_PERCENT)) / 100;
#endif
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  motordrive("backward", leftSpeed, "forward", rightSpeed);
}


void stopMotors() {
  motorsteer("stop", 0);
}

void readButton() {
  int adcValue = analogRead(A7);
  if (adcValue < 100) {
    button1 = 0;
    button2 = 1;
  } else if (adcValue < 500) {
    button1 = 1;
    button2 = 0;
  } else {
    button1 = 1;
    button2 = 1;
  }
}

int getButtonEvt(int idx) {
  return idx == 1 ? button1 : button2;
}

void performCalibration() {
  // calibrate
  digitalWrite(STATUS_LED, HIGH);
  
  qtr.resetCalibration();
  
  for (uint16_t i = 0; i < 25; i++) {
    turnCW(90, 90);
    qtr.calibrate();
    delay(10);
  }
  
  for (uint16_t i = 0; i < 25; i++) {
    turnCCW(90, 90);
    qtr.calibrate();
    delay(10);
  }
  
  stopMotors();
  storeSensorValue();
  digitalWrite(STATUS_LED, LOW);
  
  Serial.println("========================================");
  Serial.println("CALIBRATION COMPLETE!");
  Serial.println("========================================");
  
  isCalibrated = true;
}

void waitForStart() {
  Serial.println("Press Button 1 to change mode.");
  Serial.println("Press Button 2 to start.");

  for (int i = 0; i <= aggressionLevel; i++) {
    digitalWrite(MODE_LED, HIGH); delay(150);
    digitalWrite(MODE_LED, LOW); delay(150);
  }

  while (true) {
    readButton();

    if (getButtonEvt(1) == 0) {
      aggressionLevel = (aggressionLevel + 1) % 3;
      
      Kp = configs[aggressionLevel].Kp;
      Ki = configs[aggressionLevel].Ki;
      Kd = configs[aggressionLevel].Kd;
      BaseSpeed = configs[aggressionLevel].baseSpeed; 
      MaxSpeed = configs[aggressionLevel].maxSpeed;

      Serial.print("Mode changed to: ");
      if(aggressionLevel == 0) Serial.println("CONSERVATIVE");
      if(aggressionLevel == 1) Serial.println("BALANCED");
      if(aggressionLevel == 2) Serial.println("AGGRESSIVE");

      for (int i = 0; i <= aggressionLevel; i++) {
        digitalWrite(MODE_LED, HIGH); delay(150);
        digitalWrite(MODE_LED, LOW); delay(150);
      }
      delay(300);
    }

    if (getButtonEvt(2) == 0) {
      Serial.println("========================================");
      Serial.println("STARTING LINE TRACING IN 3 SECONDS...");
      Serial.println("========================================");
      digitalWrite(MODE_LED, HIGH);
      delay(3000);
      isRunning = true;
      break;
    }
    
    digitalWrite(STATUS_LED, (millis()/500) % 2);
    delay(20);
  }
  digitalWrite(STATUS_LED, LOW);
}

void saveValue(int addr, uint16_t value) {
  EEPROM.write(addr, value >> 8);
  EEPROM.write(addr + 1, value & 0xFF);
}

uint16_t readValue(int addr) {
  return (EEPROM.read(addr) << 8) + EEPROM.read(addr + 1);
}

void restoreSensorValue() {
  qtr.calibrate();
  for (int i = 0; i < SENSOR_CNT; i++) {
    int offset = i * 2;
    qtr.calibrationOn.minimum[i] = readValue(ADDR_BASE + offset);
    qtr.calibrationOn.maximum[i] = readValue(ADDR_BASE * 5 + offset);
    sensAvg[i] = (qtr.calibrationOn.maximum[i] + qtr.calibrationOn.minimum[i]) / 2;

    if (isDebug) {
      Serial.print(i);
      Serial.print(" - Min: ");
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(", Max: ");
      Serial.println(qtr.calibrationOn.maximum[i]);
    }
  }
}

void storeSensorValue() {
  for (int i = 0; i < SENSOR_CNT; i++) {
    int offset = i * 2;
    saveValue(ADDR_BASE + offset, qtr.calibrationOn.minimum[i]);
    saveValue(ADDR_BASE * 5 + offset, qtr.calibrationOn.maximum[i]);
    sensAvg[i] = (qtr.calibrationOn.maximum[i] + qtr.calibrationOn.minimum[i]) / 2;
  }
}

void initialize(bool debug) {
  isDebug = debug;
  
  if (isDebug) {
    Serial.begin(9600);
  }
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A6, A3, A2, A1, A0 }, SensorCount);
  qtr.releaseEmitterPins();

  restoreSensorValue();
}

uint16_t getSensorReadings() {
  qtr.readLineBlack(sensorValues);
  
  uint16_t sensorData = 0;
  
  for (int i = 0; i < SENSOR_CNT; i++) {
    if (sensorValues[i] > sensAvg[i]) {
      sensorData |= (1 << i);
    }
  }
  
  return sensorData;
}

int getCalculatedError(int mode) {
  uint16_t position;
  
  if (mode == 0 || trackType == BLACK_LINE_WHITE_TRACK) {
    position = qtr.readLineBlack(sensorValues);
  } else {
    position = qtr.readLineWhite(sensorValues);
  }
  
  int calculatedError = position - CENTER_VAL;
  
  return calculatedError;
}

bool isOutOfLine(unsigned int* sensors) {
  for (int i = 0; i < SENSOR_CNT; i++) {
    if (sensors[i] > sensAvg[i]) {
      return false;
    }
  }
  return true;
}

void readSensors() {
  uint16_t sensorData = getSensorReadings();
  
  error = getCalculatedError(0);
  
  int s0 = (sensorData & (1 << 0)) >> 0;
  int s1 = (sensorData & (1 << 1)) >> 1;
  int s2 = (sensorData & (1 << 2)) >> 2;
  int s3 = (sensorData & (1 << 3)) >> 3;
  int s4 = (sensorData & (1 << 4)) >> 4;
  
  if (s0 != s4) {
    error_dir = s0 - s4;
  }
  
  
  if (trackType == WHITE_LINE_BLACK_TRACK) {
    indicateInversionOn();
  } else {
    indicateInversionOff();
  }
  
  if (sensorData == 0b00000) {
    if (error_dir < 0) {
      error = OUT_OF_LINE_ERROR_VALUE;
    } else if (error_dir > 0) {
      error = -1 * OUT_OF_LINE_ERROR_VALUE;
    }
  }
  else if (sensorData == 0b11111) {
    
    uint16_t sensorDataAgain = getSensorReadings();
    
    if (sensorDataAgain == 0b11111) {
      
      Serial.println("STOP PATCH DETECTED - Stopping for 10 seconds");
    }
  }
  
  if (isDebug) {
    static int debugCounter = 0;
    if (++debugCounter % 10 == 0) {
      char sensorPattern[6];
      sprintf(sensorPattern, "%d%d%d%d%d", s0, s1, s2, s3, s4);
      
      Serial.print("Sensors: ");
      Serial.print(sensorPattern);
      Serial.print(" | Raw: ");
      for (int i = 0; i < SENSOR_CNT; i++) {
        Serial.print(sensorValues[i]);
        Serial.print(" ");
      }
      Serial.print("| Error: ");
      Serial.print(error);
      Serial.print(" | Dir: ");
      Serial.print(error_dir);
      Serial.print(" | Track: ");
      Serial.println(trackType == BLACK_LINE_WHITE_TRACK ? "BL/WT" : "WL/BL");
    }
  }
}

void calculatePID() {
  P = error;
  
  if (error == 0) {
    I = 0;
  } else {
    I = I + error;
  }
  
  I = constrain(I, -200, 200);
  
  D = error - previousError;
  
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  PID_value = constrain(PID_value, -MaxSpeed, MaxSpeed);
  
  previousError = error;
}

void controlMotors() {
  if (error == OUT_OF_LINE_ERROR_VALUE) {
    Serial.println("OUT OF LINE - Turning Clockwise");

    uint16_t sensorReadings = getSensorReadings();
    while (isOutOfLine(sensorValues)) {
      turnCW(BaseSpeed - leftMotorOffset, BaseSpeed - rightMotorOffset);
      sensorReadings = getSensorReadings();
    }
    
    error_dir = 0;
  }
  else if (error == (-1 * OUT_OF_LINE_ERROR_VALUE)) {
    Serial.println("OUT OF LINE - Turning Counter-Clockwise");
    
    uint16_t sensorReadings = getSensorReadings();
    while (isOutOfLine(sensorValues)) {
      turnCCW(BaseSpeed - leftMotorOffset, BaseSpeed - rightMotorOffset);
      sensorReadings = getSensorReadings();
    }
    
    error_dir = 0;
  }
  else {
   
    int leftMotorSpeed = BaseSpeed + PID_value - leftMotorOffset;
    int rightMotorSpeed = BaseSpeed - PID_value - rightMotorOffset;
    
    moveStraight(leftMotorSpeed, rightMotorSpeed);
    
    if (D != 0) {
      delay(loopDelay);
    }
  }
}

void setup() {
  initialize(true);
  init_motor();
  pinMode(STATUS_LED, OUTPUT);
  pinMode(MODE_LED, OUTPUT);
  
  Serial.println("========================================");
  Serial.println("LINE TRACER ROBOT - MULTI-MODE");
  Serial.println("========================================");

  // Set initial tuning parameters
  Kp = configs[aggressionLevel].Kp;
  Ki = configs[aggressionLevel].Ki;
  Kd = configs[aggressionLevel].Kd;
  BaseSpeed = configs[aggressionLevel].baseSpeed;
  MaxSpeed = configs[aggressionLevel].maxSpeed;

  performCalibration();

  waitForStart();
}

void loop() {
  if (isRunning) {
    
    readSensors();
    
    calculatePID();
    
    controlMotors();
    
    readButton();
    if (getButtonEvt(1) == 0 || getButtonEvt(2) == 0) {
      stopMotors();
      Serial.println("STOPPED - Reset to restart");
      while (true) { delay(1000); }
    }
  }
}