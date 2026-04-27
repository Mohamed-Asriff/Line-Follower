// -------------------------------
// Motor Driver TB6612FNG Pins
// -------------------------------
#define motorAInput1 7
#define motorAInput2 8
#define motorBInput1 5
#define motorBInput2 4
#define motorAPWM 9
#define motorBPWM 3
#define STBY      6  

// Button pin
#define BUTTON_PIN 2

// -------------------------------
// Sensor Pins
// -------------------------------
NEW SKETCH

#define NUM_SENSORS 8
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
NEW SKETCH


// filtered values used by PID
int sensorValues[NUM_SENSORS];
// raw analog readings used for junction/dead-end detection
int rawSensorValues[NUM_SENSORS];
// store last filtered pattern for dot-gap recovery
int lastFilteredValues[NUM_SENSORS];

// -------------------------------
// PID Constants
// -------------------------------
#define KP 2.2
#define KD 5.6
#define KI 0.009

int lastError = 0;
long integral = 0; // use long to avoid overflow

// Thresholds
#define WHITE_THRESHOLD 49
#define BLACK_THRESHOLD 50

// Button toggle
bool running = false;
bool lastButtonState = HIGH;

// -------------------------------
// Dot-line & Dead-end timing
// -------------------------------
unsigned long lastLineTime = 0;        // last time a black was seen (raw)
#define DOT_IGNORE_TIME 73  // ms — tuned for 3cm dot (500 RPM, 14cm wheel)
unsigned long lastNonDeadTime = 0;     // last time we saw any black (for dead-end confirm)
#define DEAD_END_TIME 75   // ms — must be > DOT_IGNORE_TIME

// -------------------------------
// Robot States
// -------------------------------
enum RobotState {LINE_FOLLOW, TURN_LEFT, TURN_RIGHT, U_TURN};
RobotState robotState = LINE_FOLLOW;

// -------------------------------
// U-Turn variables
// -------------------------------
unsigned long uTurnStartTime = 0;
const unsigned long uTurnDuration = 600; // total duration for U-turn (ms)

// -------------------------------
// Function Prototypes
// -------------------------------
void readSensors();
void stopMotors();
void PIDFollow();
void turnLeftStep();
void turnRightStep();
bool centerSensorsDetectLine_raw();
bool centerSensorsDetectLine_filtered();
bool deadEndDetected_timed();
bool searchForLine();

void setup() {
  pinMode(motorAInput1, OUTPUT);
  pinMode(motorAInput2, OUTPUT);
  pinMode(motorBInput1, OUTPUT);
  pinMode(motorBInput2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorBPWM, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorValues[i] = 0;
    rawSensorValues[i] = 0;
    lastFilteredValues[i] = 0;
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(9600);
}

// =======================================================================
// MAIN LOOP
// =======================================================================
void loop() {

  // BUTTON START/STOP
  bool currentState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && currentState == LOW) {
    running = !running;
    delay(250);
  }
  lastButtonState = currentState;

  if (!running) {
    stopMotors();
    return;
  }

  // Read sensor values (fills rawSensorValues[] and filtered sensorValues[])
  readSensors();

  // Check L junctions using RAW readings (dot-filter won't hide them)
  bool leftL = (rawSensorValues[0] > BLACK_THRESHOLD || rawSensorValues[1] > BLACK_THRESHOLD || rawSensorValues[2] > BLACK_THRESHOLD) &&
               (rawSensorValues[3] < BLACK_THRESHOLD && rawSensorValues[4] < BLACK_THRESHOLD);

  bool rightL = (rawSensorValues[5] > BLACK_THRESHOLD || rawSensorValues[6] > BLACK_THRESHOLD || rawSensorValues[7] > BLACK_THRESHOLD) &&
                (rawSensorValues[3] < BLACK_THRESHOLD && rawSensorValues[4] < BLACK_THRESHOLD);

  // State machine
  switch (robotState) {
    case LINE_FOLLOW:
      if (leftL) {
        robotState = TURN_LEFT;
      }
      else if (rightL) {
        robotState = TURN_RIGHT;
      }
      else if (deadEndDetected_timed()) {
          robotState = U_TURN;
          uTurnStartTime = millis();   // start timing the U-turn
      }
      else PIDFollow();
      break;

    case TURN_LEFT:
      turnLeftStep();
      // resume when center sensors detect line (raw or filtered)
      if (centerSensorsDetectLine_raw() || centerSensorsDetectLine_filtered()) robotState = LINE_FOLLOW;
      break;

    case TURN_RIGHT:
      turnRightStep();
      if (centerSensorsDetectLine_raw() || centerSensorsDetectLine_filtered()) robotState = LINE_FOLLOW;
      break;

    case U_TURN:
      // Step 1: move forward slightly (~3cm)
      if (millis() - uTurnStartTime < 200) {  // adjust 200ms for ~3cm
        digitalWrite(motorAInput1, LOW);
        digitalWrite(motorAInput2, HIGH);
        digitalWrite(motorBInput1, LOW);
        digitalWrite(motorBInput2, HIGH);
        analogWrite(motorAPWM, 120);
        analogWrite(motorBPWM, 120);
      }
      // Step 2: sharp 180° turn (left motor stopped, right motor moves)
      else if (millis() - uTurnStartTime < uTurnDuration + 200) {
        digitalWrite(motorAInput1, LOW);  // Left motor stopped
        digitalWrite(motorAInput2, LOW);
        digitalWrite(motorBInput1, HIGH); // Right motor forward
        digitalWrite(motorBInput2, LOW);
        analogWrite(motorAPWM, 0);
        analogWrite(motorBPWM, 150);
      }
      // Step 3: search for line
      else {
        if (searchForLine()) {
          robotState = LINE_FOLLOW; // resume line following when line found
        }
      }
      break;
  }
}

// =======================================================================
// READ SENSOR FUNCTION (RAW + FILTERED with DOT-LINE IGNORE)
// =======================================================================
void readSensors() {

  int sensorSumRaw = 0;

  // 1) read raw analog values
  for (int i = 0; i < NUM_SENSORS; i++) {
    rawSensorValues[i] = analogRead(sensorPins[i]);
    if (rawSensorValues[i] > BLACK_THRESHOLD) sensorSumRaw++;
  }

  // If any raw black → update lastLineTime and update filtered values
  if (sensorSumRaw > 0) {
    lastLineTime = millis();
    lastNonDeadTime = millis(); // seen black, reset dead-end timer

    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = rawSensorValues[i];
      lastFilteredValues[i] = sensorValues[i]; // store last good filtered pattern
    }
    return;
  }

  // No raw black detected (all white)
  // If this white gap is short (< DOT_IGNORE_TIME) → reuse last filtered pattern for PID
  if (millis() - lastLineTime < DOT_IGNORE_TIME) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = lastFilteredValues[i];
    }
    return;
  }

  // Otherwise, full-white (no line) → filtered = raw (all white)
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = rawSensorValues[i];
  }
}

// =======================================================================
// PID LINE FOLLOWING (uses filtered sensorValues[])
// =======================================================================
void PIDFollow() {
  long weightedSum = 0;
  long sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int v = sensorValues[i];
    int value;
    if (v > BLACK_THRESHOLD) value = 1000;
    else if (v < WHITE_THRESHOLD) value = 0;
    else value = map(v, WHITE_THRESHOLD, BLACK_THRESHOLD, 0, 1000);

    weightedSum += (long)(i * 1000) * value;
    sum += value;
  }

  int position = (sum != 0) ? weightedSum / sum : (NUM_SENSORS - 1) * 500;
  int error = position - ((NUM_SENSORS - 1) * 500);

  int proportional = (int)(KP * error);
  int derivative  = (int)(KD * (error - lastError));
  integral += error;
  int integralComp = (int)(KI * integral);

  int controlSignal = proportional + derivative + integralComp;

  int baseSpeed = 190;
  int motorSpeedA = baseSpeed - controlSignal / 43;
  int motorSpeedB = baseSpeed + controlSignal / 43;

  motorSpeedA = constrain(motorSpeedA, 0, 255);
  motorSpeedB = constrain(motorSpeedB, 0, 255);

  digitalWrite(motorAInput1, LOW);
  digitalWrite(motorAInput2, HIGH);
  digitalWrite(motorBInput1, LOW);
  digitalWrite(motorBInput2, HIGH);

  analogWrite(motorAPWM, motorSpeedA);
  analogWrite(motorBPWM, motorSpeedB);

  lastError = error;
}

// =======================================================================
// TURN LEFT STEP
// =======================================================================
void turnLeftStep() {
  digitalWrite(motorAInput1, LOW);
  digitalWrite(motorAInput2, HIGH);
  digitalWrite(motorBInput1, HIGH);
  digitalWrite(motorBInput2, LOW);

  analogWrite(motorAPWM, 120);
  analogWrite(motorBPWM, 120);
}

// =======================================================================
// TURN RIGHT STEP
// =======================================================================
void turnRightStep() {
  digitalWrite(motorAInput1, HIGH);
  digitalWrite(motorAInput2, LOW);
  digitalWrite(motorBInput1, LOW);
  digitalWrite(motorBInput2, HIGH);

  analogWrite(motorAPWM, 120);
  analogWrite(motorBPWM, 120);
}

// =======================================================================
// CENTER SENSOR DETECTION (RAW and FILTERED versions)
// =======================================================================
bool centerSensorsDetectLine_raw() {
  return (rawSensorValues[3] > BLACK_THRESHOLD || rawSensorValues[4] > BLACK_THRESHOLD);
}
bool centerSensorsDetectLine_filtered() {
  return (sensorValues[3] > BLACK_THRESHOLD || sensorValues[4] > BLACK_THRESHOLD);
}

// =======================================================================
// DEAD-END DETECTION (timed to avoid dot-line false positives)
// =======================================================================
bool deadEndDetected_timed() {
  // If any raw sensor sees black, update lastNonDeadTime and return false
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (rawSensorValues[i] > BLACK_THRESHOLD) {
      lastNonDeadTime = millis();
      return false;
    }
  }
  // all raw sensors white - only consider dead end if lasted longer than DEAD_END_TIME
  if (millis() - lastNonDeadTime > DEAD_END_TIME) return true;
  return false;
}

// =======================================================================
// SEARCH FOR LINE AFTER U-TURN
// =======================================================================
bool searchForLine() {
  readSensors();
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (rawSensorValues[i] > BLACK_THRESHOLD) return true;
  }

  // Rotate slowly to find line
  digitalWrite(motorAInput1, LOW);
  digitalWrite(motorAInput2, HIGH);
  digitalWrite(motorBInput1, HIGH);
  digitalWrite(motorBInput2, LOW);
  analogWrite(motorAPWM, 150);
  analogWrite(motorBPWM, 150);

  return false;
}

// =======================================================================
// STOP MOTORS FUNCTION
// =======================================================================
void stopMotors() {
  digitalWrite(motorAInput1, LOW);
  digitalWrite(motorAInput2, LOW);
  digitalWrite(motorBInput1, LOW);
  digitalWrite(motorBInput2, LOW);
  analogWrite(motorAPWM, 0);
  analogWrite(motorBPWM, 0);
}
