#include <TektiteRotEv.h>
#include <math.h>

RotEv rotev;

// --------------------------------------------------
// Route step
// --------------------------------------------------
struct MoveStep {
float x;
float y;
bool reverse;
};

#define STEP(x,y,rev) {x,y,rev}

// --------------------------------------------------
// Robot constants
// --------------------------------------------------
const float WHEEL_DIAMETER_CM = 6.03f;
const float WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * 3.14159f;

const float DEG_TO_RAD_F = 3.14159f / 180.0f;
const float RAD_TO_DEG_F = 180.0f / 3.14159f;

// --------------------------------------------------
// Grid coordinate system
// x = right positive, left negative
// y = forward positive, back negative
// --------------------------------------------------
float X_STEP_CM = 50.0f;
float Y_STEP_CM = 50.0f;

// --------------------------------------------------
// Drive tuning
// --------------------------------------------------
const float MAX_CRUISE_CMPS = 70.0f;
const float MAX_ACCEL_CMPS2 = 325.0f;
const float MAX_DECEL_CMPS2 = 130.0f;
const float MIN_PROFILE_CMPS = 4.0f;

const float kP_speed = 0.525f;
const float kStaticFwd = 1.3f;
const float SPEED_ERROR_TOL_CMPS = 1.5f;

// --------------------------------------------------
// Heading hold while driving
// --------------------------------------------------
const float kP_heading = 0.28f;
const float kD_heading = 0.015f;
const float HEADING_TOL_DEG = 0.1f;
const float kStaticHeadingTurn = 0.7f;

// --------------------------------------------------
// Turn-only PD
// --------------------------------------------------
const float kP_turn = 0.3f;
const float kD_turn = 0.012f;
const float TURN_TOL_DEG = 1.0f;
const float TURN_RATE_TOL_DPS = 6.0f;
const float kStaticTurn = 0.935f;

const float TURN_BREAKAWAY_START_VOLTS = 0.935f;
const float TURN_BREAKAWAY_STEP_VOLTS = 0.05f;
const float TURN_BREAKAWAY_MAX_VOLTS = 1.6f;
const float TURN_BREAKAWAY_YAW_DPS = 1.0f;

// --------------------------------------------------
// Output caps
// --------------------------------------------------
const float MAX_FWD_VOLTS = 8.0f;
const float MAX_TURN_VOLTS = 1.5f;
const float MAX_TURN_ONLY_VOLTS = 3.0f;

// --------------------------------------------------
// Exit thresholds
// --------------------------------------------------
const float DIST_TOL_CM = 1.0f;
const float LIN_VEL_TOL_CMPS = 1.0f;

// --------------------------------------------------
// Delay pacing / stop safety
// --------------------------------------------------
const float TARGET_RUN_TIME_SEC = 75.0f;
const unsigned long MAX_WAIT_MS = 1500;
const unsigned long DRIVE_TIMEOUT_MS = 12000;
const unsigned long TURN_TIMEOUT_MS = 40000;

const unsigned long MAX_TOTAL_STOP_MS = 1650;
const unsigned long STOP_SAFETY_MARGIN_MS = 100;

// --------------------------------------------------
// Periodic gyro recalibration
// --------------------------------------------------
const unsigned long PERIODIC_RECAL_INTERVAL_MS = 5000;
const unsigned long PERIODIC_RECAL_MS = 2000;
const float PERIODIC_RECAL_MAX_LINVEL_CMPS = 0.3f;
const float PERIODIC_RECAL_MAX_YAW_DPS = 0.5f;

unsigned long runStartMs = 0;
unsigned long lastPeriodicRecalMs = 0;

// --------------------------------------------------
// Velocity filter
// --------------------------------------------------
const float VEL_FILTER_ALPHA = 0.5f;

// --------------------------------------------------
// Gyro cleanup
// --------------------------------------------------
const int CAL_SAMPLES = 2000;
const float YAW_DEADBAND_RAD = 0.01f;

// --------------------------------------------------
// Debug
// --------------------------------------------------
const bool DEBUG_SERIAL = true;
const unsigned long DEBUG_PRINT_MS = 80;
const unsigned long POSE_PRINT_MS = 120;

unsigned long lastDebugPrintMs = 0;
unsigned long lastPosePrintMs = 0;
unsigned long lastLoopMicros = 0;

// --------------------------------------------------
// Globals
// --------------------------------------------------
float lastEnc1 = 0.0f;
float lastEnc2 = 0.0f;
float enc1AccumDeg = 0.0f;
float enc2AccumDeg = 0.0f;

float gyroBiasRad = 0.0f;
float headingRad = 0.0f;
float filteredYawRad = 0.0f;

float vel1 = 0.0f;
float vel2 = 0.0f;

// Odom in robot/world cm frame:
// posX_cm = forward positive
// posY_cm = left positive
float posX_cm = 0.0f;
float posY_cm = 0.0f;

// how long the robot has already effectively been stopped
// in the current stop stretch after the most recent action
unsigned long lastInactiveMs = 0;

// --------------------------------------------------
// Helpers
// --------------------------------------------------
float clampValue(float x, float low, float high) {
if (x < low) return low;
if (x > high) return high;
return x;
}

float signOf(float x) {
if (x > 0) return 1.0f;
if (x < 0) return -1.0f;
return 0.0f;
}

float unwrapDelta(float current, float previous) {
float delta = current - previous;
if (delta > 180.0f) delta -= 360.0f;
if (delta < -180.0f) delta += 360.0f;
return delta;
}

float wrapAngleRad(float angle) {
while (angle > 3.14159f) angle -= 2.0f * 3.14159f;
while (angle < -3.14159f) angle += 2.0f * 3.14159f;
return angle;
}

float wrapAngleDeg(float angle) {
while (angle > 180.0f) angle -= 360.0f;
while (angle < -180.0f) angle += 360.0f;
return angle;
}

float lerp(float a, float b, float t) {
return a + (b - a) * t;
}

void stopRobot() {
rotev.motorWrite1(0.0f);
rotev.motorWrite2(0.0f);
}

void setMotorVolts(float leftVolts, float rightVolts) {
float vbat = rotev.getVoltage();
if (vbat < 1.0f) {
stopRobot();
return;
}

float leftDuty = clampValue(leftVolts / vbat, -1.0f, 1.0f);
float rightDuty = clampValue(rightVolts / vbat, -1.0f, 1.0f);

rotev.motorWrite1(-leftDuty);
rotev.motorWrite2(rightDuty);
}

// --------------------------------------------------
// Grid conversion helpers
// --------------------------------------------------
float gridXToRobotYcm(float x) {
return -x * X_STEP_CM;
}

float gridYToRobotXcm(float y) {
return y * Y_STEP_CM;
}

float robotYcmToGridX(float robotY_cm) {
return -robotY_cm / X_STEP_CM;
}

float robotXcmToGridY(float robotX_cm) {
return robotX_cm / Y_STEP_CM;
}

// --------------------------------------------------
// Debug helpers
// --------------------------------------------------
void debugPrintPose() {
if (!DEBUG_SERIAL) return;
if (millis() - lastPosePrintMs < POSE_PRINT_MS) return;
lastPosePrintMs = millis();

Serial.print("[POSE] x_cm=");
Serial.print(posX_cm, 2);
Serial.print(" y_cm=");
Serial.print(posY_cm, 2);
Serial.print(" head=");
Serial.print(headingRad * RAD_TO_DEG_F, 2);
Serial.print(" gx=");
Serial.print(robotYcmToGridX(posY_cm), 2);
Serial.print(" gy=");
Serial.println(robotXcmToGridY(posX_cm), 2);
}

void debugPrintTurnStatus(float targetHeadingDeg,
float headingErrorDeg,
float yawRateDegPerSec,
float turnVolts,
bool angleDone,
bool rateDone,
unsigned long elapsedMs) {
if (!DEBUG_SERIAL) return;
if (millis() - lastDebugPrintMs < DEBUG_PRINT_MS) return;
lastDebugPrintMs = millis();

Serial.print("[TURN] t=");
Serial.print(elapsedMs);
Serial.print(" tgt=");
Serial.print(targetHeadingDeg, 2);
Serial.print(" head=");
Serial.print(headingRad * RAD_TO_DEG_F, 2);
Serial.print(" err=");
Serial.print(headingErrorDeg, 2);
Serial.print(" yaw=");
Serial.print(yawRateDegPerSec, 2);
Serial.print(" V=");
Serial.print(turnVolts, 2);
Serial.print(" waitAngle=");
Serial.print(angleDone ? 0 : 1);
Serial.print(" waitRate=");
Serial.println(rateDone ? 0 : 1);
}

void debugPrintDriveStatus(float targetCm,
float dist,
float distError,
float targetSpeed,
float linVelCmPerSec,
float headingErrorDeg,
float yawRateDegPerSec,
float forwardVolts,
float turnVolts,
bool distDone,
bool velDone,
unsigned long elapsedMs) {
if (!DEBUG_SERIAL) return;
if (millis() - lastDebugPrintMs < DEBUG_PRINT_MS) return;
lastDebugPrintMs = millis();

Serial.print("[DRIVE] t=");
Serial.print(elapsedMs);
Serial.print(" tgt=");
Serial.print(targetCm, 2);
Serial.print(" dist=");
Serial.print(dist, 2);
Serial.print(" err=");
Serial.print(distError, 2);
Serial.print(" targSpd=");
Serial.print(targetSpeed, 2);
Serial.print(" vel=");
Serial.print(linVelCmPerSec, 2);
Serial.print(" head=");
Serial.print(headingRad * RAD_TO_DEG_F, 2);
Serial.print(" hErr=");
Serial.print(headingErrorDeg, 2);
Serial.print(" yaw=");
Serial.print(yawRateDegPerSec, 2);
Serial.print(" fwdV=");
Serial.print(forwardVolts, 2);
Serial.print(" turnV=");
Serial.print(turnVolts, 2);
Serial.print(" waitDist=");
Serial.print(distDone ? 0 : 1);
Serial.print(" waitVel=");
Serial.println(velDone ? 0 : 1);
}

// --------------------------------------------------
// Gyro
// --------------------------------------------------
void sortArray(float* arr, int n) {
for (int i = 0; i < n - 1; i++) {
for (int j = i + 1; j < n; j++) {
if (arr[j] < arr[i]) {
float t = arr[i];
arr[i] = arr[j];
arr[j] = t;
}
}
}
}

void calibrateGyroBias() {
stopRobot();
static float samples[CAL_SAMPLES];

for (int i = 0; i < CAL_SAMPLES; i++) {
samples[i] = rotev.readYawRate();
delay(1);
}

sortArray(samples, CAL_SAMPLES);

int trim = CAL_SAMPLES / 10;
float total = 0.0f;
int count = 0;

for (int i = trim; i < CAL_SAMPLES - trim; i++) {
total += samples[i];
count++;
}

gyroBiasRad = total / count;
filteredYawRad = 0.0f;
headingRad = 0.0f;

if (DEBUG_SERIAL) {
Serial.print("gyroBias(rad/s) = ");
Serial.println(gyroBiasRad, 6);
Serial.print("gyroBias(deg/s) = ");
Serial.println(gyroBiasRad * RAD_TO_DEG_F, 4);
}
}

void resetPoseCm(float startX_cm = 0.0f, float startY_cm = 0.0f, float startHeadingDeg = 0.0f) {
posX_cm = startY_cm;
posY_cm = -startX_cm;
headingRad = startHeadingDeg * DEG_TO_RAD_F;
filteredYawRad = 0.0f;
}

float averageVelocityCmPerSec();

float updateHeading(float dt) {
float rawYawRad = rotev.readYawRate();
float correctedYawRad = rawYawRad - gyroBiasRad;

filteredYawRad = correctedYawRad;

if (fabs(filteredYawRad) < YAW_DEADBAND_RAD) {
filteredYawRad = 0.0f;
}

headingRad += filteredYawRad * dt;
headingRad = wrapAngleRad(headingRad);

return filteredYawRad;
}

// --------------------------------------------------
// Encoder tracking + filtered wheel velocity
// --------------------------------------------------
void resetEncodersTracked() {
lastEnc1 = rotev.enc1AngleDegrees();
lastEnc2 = rotev.enc2AngleDegrees();
enc1AccumDeg = 0.0f;
enc2AccumDeg = 0.0f;
vel1 = 0.0f;
vel2 = 0.0f;
}

void updateEncodersTracked(float dt) {
float now1 = rotev.enc1AngleDegrees();
float now2 = rotev.enc2AngleDegrees();

float d1Deg = unwrapDelta(now1, lastEnc1);
float d2Deg = unwrapDelta(now2, lastEnc2);

enc1AccumDeg += d1Deg;
enc2AccumDeg += d2Deg;

lastEnc1 = now1;
lastEnc2 = now2;

float leftDeltaCm = (-d1Deg / 360.0f) * WHEEL_CIRCUMFERENCE_CM;
float rightDeltaCm = (d2Deg / 360.0f) * WHEEL_CIRCUMFERENCE_CM;

float vel1_raw = leftDeltaCm / dt;
float vel2_raw = rightDeltaCm / dt;

vel1 = (1.0f - VEL_FILTER_ALPHA) * vel1 + VEL_FILTER_ALPHA * vel1_raw;
vel2 = (1.0f - VEL_FILTER_ALPHA) * vel2 + VEL_FILTER_ALPHA * vel2_raw;

float deltaCenterCm = 0.5f * (leftDeltaCm + rightDeltaCm);
posX_cm += deltaCenterCm * cosf(headingRad);
posY_cm += deltaCenterCm * sinf(headingRad);

debugPrintPose();
}

float leftDistanceCm() {
return (-enc1AccumDeg / 360.0f) * WHEEL_CIRCUMFERENCE_CM;
}

float rightDistanceCm() {
return (enc2AccumDeg / 360.0f) * WHEEL_CIRCUMFERENCE_CM;
}

float averageDistanceCm() {
return (leftDistanceCm() + rightDistanceCm()) * 0.5f;
}

float averageVelocityCmPerSec() {
return (vel1 + vel2) * 0.5f;
}

// --------------------------------------------------
// Point helpers
// --------------------------------------------------
float angleToPointDeg(float targetX_cm, float targetY_cm) {
float dx = targetX_cm - posX_cm;
float dy = targetY_cm - posY_cm;
return atan2f(dy, dx) * RAD_TO_DEG_F;
}

float distanceToPointCm(float targetX_cm, float targetY_cm) {
float dx = targetX_cm - posX_cm;
float dy = targetY_cm - posY_cm;
return sqrtf(dx * dx + dy * dy);
}

// --------------------------------------------------
// Buttons
// --------------------------------------------------
void waitForGoRelease() {
while (rotev.goButtonPressed()) {
delay(10);
}
}

void waitForFreshGoPress() {
while (!rotev.goButtonPressed()) {
if (rotev.stopButtonPressed()) {
stopRobot();
}
delay(10);
}
}

// --------------------------------------------------
// Stop-budget helpers
// --------------------------------------------------
unsigned long capExtraStopMs(unsigned long requestedMs, unsigned long alreadyStoppedMs) {
long allowedMs = (long)MAX_TOTAL_STOP_MS - (long)alreadyStoppedMs - (long)STOP_SAFETY_MARGIN_MS;

if (allowedMs < 0) allowedMs = 0;
if (requestedMs > (unsigned long)allowedMs) {
requestedMs = (unsigned long)allowedMs;
}

return requestedMs;
}

unsigned long capWaitByLastInactive(unsigned long requestedWaitMs) {
unsigned long cappedMs = capExtraStopMs(requestedWaitMs, lastInactiveMs);

if (DEBUG_SERIAL) {
Serial.print("[WAIT CAP] lastInactive=");
Serial.print(lastInactiveMs);
Serial.print(" requested=");
Serial.print(requestedWaitMs);
Serial.print(" allowed=");
Serial.println(cappedMs);
}

return cappedMs;
}

void doDelayMs(unsigned long delayMs) {
unsigned long startMs = millis();
while (millis() - startMs < delayMs) {
if (rotev.stopButtonPressed()) {
stopRobot();
return;
}
delay(1);
}
}

bool doPeriodicRecalIfDue() {
if (millis() - lastPeriodicRecalMs < PERIODIC_RECAL_INTERVAL_MS) {
return false;
}

stopRobot();

float yawDps = filteredYawRad * RAD_TO_DEG_F;
float linVel = averageVelocityCmPerSec();

if (fabs(linVel) > PERIODIC_RECAL_MAX_LINVEL_CMPS ||
fabs(yawDps) > PERIODIC_RECAL_MAX_YAW_DPS) {
if (DEBUG_SERIAL) {
Serial.print("[RECAL] skipped linVel=");
Serial.print(linVel, 2);
Serial.print(" yaw=");
Serial.println(yawDps, 2);
}
return false;
}

unsigned long recalMs = capExtraStopMs(PERIODIC_RECAL_MS, lastInactiveMs);
if (recalMs == 0) {
if (DEBUG_SERIAL) {
Serial.println("[RECAL] skipped no stop budget");
}
return false;
}

if (DEBUG_SERIAL) {
Serial.print("[RECAL] start ms=");
Serial.println(recalMs);
}

unsigned long startMs = millis();
float total = 0.0f;
int count = 0;

while (millis() - startMs < recalMs) {
if (rotev.stopButtonPressed()) {
stopRobot();
return false;
}

float rawYawRad = rotev.readYawRate();
total += rawYawRad;
count++;
delay(1);
}

if (count > 0) {
gyroBiasRad = total / count;
}

filteredYawRad = 0.0f;
lastInactiveMs += recalMs;
lastPeriodicRecalMs = millis();

if (DEBUG_SERIAL) {
Serial.print("[RECAL] new bias deg/s = ");
Serial.println(gyroBiasRad * RAD_TO_DEG_F, 4);
}

return true;
}

// --------------------------------------------------
// Turning PD
// --------------------------------------------------
bool turnToHeadingDeg(float targetHeadingDeg) {
unsigned long turnStartMs = millis();
unsigned long inactiveStartMs = 0;
unsigned long maxInactiveMs = 0;
bool inactiveNow = false;
float breakawayVolts = TURN_BREAKAWAY_START_VOLTS;

lastInactiveMs = 0;
lastLoopMicros = micros();
lastDebugPrintMs = 0;

Serial.print("TURN START -> ");
Serial.println(targetHeadingDeg, 2);

while (true) {
unsigned long nowMicros = micros();
while (nowMicros - lastLoopMicros < 1000) {
nowMicros = micros();
}

float dt = (nowMicros - lastLoopMicros) / 1000000.0f;
lastLoopMicros = nowMicros;

if (rotev.stopButtonPressed()) {
Serial.println("TURN ABORT: stop button");
stopRobot();
lastInactiveMs = maxInactiveMs;
return false;
}

float yawRateRadPerSec = updateHeading(dt);
float yawRateDegPerSec = yawRateRadPerSec * RAD_TO_DEG_F;

bool currentlyInactive = fabs(yawRateDegPerSec) < 1.0f;
if (currentlyInactive) {
if (!inactiveNow) {
inactiveNow = true;
inactiveStartMs = millis();
}
unsigned long currInactiveMs = millis() - inactiveStartMs;
if (currInactiveMs > maxInactiveMs) {
maxInactiveMs = currInactiveMs;
}
} else {
inactiveNow = false;
}

float headingErrorRad = wrapAngleRad(targetHeadingDeg * DEG_TO_RAD_F - headingRad);
float headingErrorDeg = headingErrorRad * RAD_TO_DEG_F;

float turnVolts = kP_turn * headingErrorDeg - kD_turn * yawRateDegPerSec;

if (fabs(headingErrorDeg) < TURN_TOL_DEG) {
turnVolts = 0.0f;
breakawayVolts = TURN_BREAKAWAY_START_VOLTS;
} else {
if (fabs(turnVolts) < breakawayVolts) {
turnVolts = signOf(turnVolts) * breakawayVolts;
}

if (fabs(yawRateDegPerSec) < TURN_BREAKAWAY_YAW_DPS &&
fabs(turnVolts) >= breakawayVolts - 0.001f) {
breakawayVolts += TURN_BREAKAWAY_STEP_VOLTS;
if (breakawayVolts > TURN_BREAKAWAY_MAX_VOLTS) {
breakawayVolts = TURN_BREAKAWAY_MAX_VOLTS;
}
} else {
breakawayVolts = TURN_BREAKAWAY_START_VOLTS;
}
}

turnVolts = clampValue(turnVolts, -MAX_TURN_ONLY_VOLTS, MAX_TURN_ONLY_VOLTS);
setMotorVolts(-turnVolts, turnVolts);

bool angleDone = fabs(headingErrorDeg) < TURN_TOL_DEG;
bool rateDone = fabs(yawRateDegPerSec) < TURN_RATE_TOL_DPS;

debugPrintTurnStatus(
targetHeadingDeg,
headingErrorDeg,
yawRateDegPerSec,
turnVolts,
angleDone,
rateDone,
millis() - turnStartMs
);

if (angleDone && rateDone) {
break;
}

if (millis() - turnStartMs > TURN_TIMEOUT_MS) {
Serial.println("TURN TIMEOUT");
stopRobot();
lastInactiveMs = maxInactiveMs;
return false;
}
}

stopRobot();

unsigned long postTurnDelayMs = 20;
postTurnDelayMs = capExtraStopMs(postTurnDelayMs, maxInactiveMs);
delay(postTurnDelayMs);

filteredYawRad = 0.0f;
lastInactiveMs = maxInactiveMs + postTurnDelayMs;

Serial.println("TURN DONE");
return true;
}

// --------------------------------------------------
// Trapezoidal straight drive
// --------------------------------------------------
bool driveDistanceCmProfile(float targetCm, float targetHeadingDeg) {
unsigned long driveStartMs = millis();
unsigned long inactiveStartMs = 0;
unsigned long maxInactiveMs = 0;
bool inactiveNow = false;

lastInactiveMs = 0;
lastDebugPrintMs = 0;
resetEncodersTracked();
lastLoopMicros = micros();

Serial.print("DRIVE START dist=");
Serial.print(targetCm, 2);
Serial.print(" head=");
Serial.println(targetHeadingDeg, 2);

while (true) {
unsigned long nowMicros = micros();
while (nowMicros - lastLoopMicros < 1000) {
nowMicros = micros();
}

float dt = (nowMicros - lastLoopMicros) / 1000000.0f;
lastLoopMicros = nowMicros;

if (rotev.stopButtonPressed()) {
Serial.println("DRIVE ABORT: stop button");
stopRobot();
lastInactiveMs = maxInactiveMs;
return false;
}

float yawRateRadPerSec = updateHeading(dt);
float yawRateDegPerSec = yawRateRadPerSec * RAD_TO_DEG_F;

updateEncodersTracked(dt);

float dist = averageDistanceCm();
float distError = targetCm - dist;
float linVelCmPerSec = averageVelocityCmPerSec();

bool currentlyInactive =
fabs(linVelCmPerSec) < 1.0f &&
fabs(yawRateDegPerSec) < 1.0f;

if (currentlyInactive) {
if (!inactiveNow) {
inactiveNow = true;
inactiveStartMs = millis();
}
unsigned long currInactiveMs = millis() - inactiveStartMs;
if (currInactiveMs > maxInactiveMs) {
maxInactiveMs = currInactiveMs;
}
} else {
inactiveNow = false;
}

float distTraveled = fabs(dist);
float distRemaining = fabs(distError);

float accelLimitedSpeed = sqrtf(2.0f * MAX_ACCEL_CMPS2 * distTraveled);
float decelLimitedSpeed = sqrtf(2.0f * MAX_DECEL_CMPS2 * distRemaining);
float targetSpeedAbs = fminf(MAX_CRUISE_CMPS, fminf(accelLimitedSpeed, decelLimitedSpeed));

if (distRemaining > DIST_TOL_CM && targetSpeedAbs < MIN_PROFILE_CMPS) {
targetSpeedAbs = MIN_PROFILE_CMPS;
}

float targetSpeed = signOf(distError) * targetSpeedAbs;

float speedError = targetSpeed - linVelCmPerSec;
if (fabs(speedError) < SPEED_ERROR_TOL_CMPS) {
speedError = 0.0f;
}

float forwardVolts = kP_speed * speedError;

bool nearStopped = fabs(linVelCmPerSec) < 2.0f;
bool wantMove = fabs(targetSpeed) > 1.0f;
bool sameDirection =
(targetSpeed > 0.0f && linVelCmPerSec >= -0.5f) ||
(targetSpeed < 0.0f && linVelCmPerSec <= 0.5f);

if (distRemaining > DIST_TOL_CM &&
wantMove &&
nearStopped &&
sameDirection &&
fabs(forwardVolts) < kStaticFwd) {
forwardVolts = signOf(targetSpeed) * kStaticFwd;
}

float headingErrorRad = wrapAngleRad(targetHeadingDeg * DEG_TO_RAD_F - headingRad);
float headingErrorDeg = headingErrorRad * RAD_TO_DEG_F;

if (fabs(headingErrorDeg) < HEADING_TOL_DEG) {
headingErrorDeg = 0.0f;
}

float turnVolts = kP_heading * headingErrorDeg - kD_heading * yawRateDegPerSec;

if (fabs(headingErrorDeg) < HEADING_TOL_DEG) {
turnVolts = 0.0f;
} else if (fabs(turnVolts) < kStaticHeadingTurn) {
turnVolts = signOf(turnVolts) * kStaticHeadingTurn;
}

forwardVolts = clampValue(forwardVolts, -MAX_FWD_VOLTS, MAX_FWD_VOLTS);
turnVolts = clampValue(turnVolts, -MAX_TURN_VOLTS, MAX_TURN_VOLTS);

float leftVolts = forwardVolts - turnVolts;
float rightVolts = forwardVolts + turnVolts;

setMotorVolts(leftVolts, rightVolts);

bool distDone = fabs(distRemaining) < DIST_TOL_CM;
bool velDone = fabs(linVelCmPerSec) < LIN_VEL_TOL_CMPS;

debugPrintDriveStatus(
targetCm,
dist,
distError,
targetSpeed,
linVelCmPerSec,
headingErrorDeg,
yawRateDegPerSec,
forwardVolts,
turnVolts,
distDone,
velDone,
millis() - driveStartMs
);

if (distDone && velDone) {
break;
}

if (millis() - driveStartMs > DRIVE_TIMEOUT_MS) {
Serial.println("DRIVE TIMEOUT");
stopRobot();
lastInactiveMs = maxInactiveMs;
return false;
}
}

stopRobot();

unsigned long postDriveDelayMs = 30;
postDriveDelayMs = capExtraStopMs(postDriveDelayMs, maxInactiveMs);
delay(postDriveDelayMs);

lastInactiveMs = maxInactiveMs + postDriveDelayMs;

Serial.println("DRIVE DONE");
return true;
}

// --------------------------------------------------
// Delay estimator
// --------------------------------------------------
float estimateTurnTimeSecFromAngle(float angleDeg) {
angleDeg = fabs(angleDeg);

if (angleDeg <= 45.0f) {
float frac = angleDeg / 45.0f;
return lerp(0.0f, 0.559f, frac);
}
if (angleDeg <= 90.0f) {
float frac = (angleDeg - 45.0f) / 45.0f;
return lerp(0.559f, 0.634f, frac);
}
if (angleDeg <= 135.0f) {
float frac = (angleDeg - 90.0f) / 45.0f;
return lerp(0.634f, 0.729f, frac);
}
if (angleDeg <= 180.0f) {
float frac = (angleDeg - 135.0f) / 45.0f;
return lerp(0.729f, 0.834f, frac);
}

return 0.834f;
}

float estimateDriveTimeSecFromDistance(float distanceCm) {
distanceCm = fabs(distanceCm);

if (distanceCm <= 25.0f) {
float frac = distanceCm / 25.0f;
return lerp(0.0f, 0.942f, frac);
}
if (distanceCm <= 50.0f) {
float frac = (distanceCm - 25.0f) / 25.0f;
return lerp(0.942f, 1.269f, frac);
}
if (distanceCm <= 78.0f) {
float frac = (distanceCm - 50.0f) / 28.0f;
return lerp(1.269f, 1.619f, frac);
}
if (distanceCm <= 100.0f) {
float frac = (distanceCm - 78.0f) / 22.0f;
return lerp(1.619f, 1.888f, frac);
}

float slope = (1.888f - 1.619f) / (100.0f - 78.0f);
return 1.888f + (distanceCm - 100.0f) * slope;
}

float targetHeadingForMove(float fromRobotX_cm, float fromRobotY_cm,
float toRobotX_cm, float toRobotY_cm,
bool reverse) {
float dx = toRobotX_cm - fromRobotX_cm;
float dy = toRobotY_cm - fromRobotY_cm;
float headingDeg = atan2f(dy, dx) * RAD_TO_DEG_F;

if (reverse) {
headingDeg += 180.0f;
headingDeg = wrapAngleDeg(headingDeg);
}

return headingDeg;
}

float angleDiffDeg(float a, float b) {
return wrapAngleDeg(a - b);
}

float estimateRemainingMotionTimeSec(const MoveStep* steps, int count) {
if (count <= 0) return 0.0f;

float prevRobotX_cm = posX_cm;
float prevRobotY_cm = posY_cm;
float prevHeadingDeg = headingRad * RAD_TO_DEG_F;

float totalSec = 0.0f;

for (int i = 0; i < count; i++) {
float targetRobotX_cm = gridYToRobotXcm(steps[i].y);
float targetRobotY_cm = gridXToRobotYcm(steps[i].x);

float moveHeadingDeg = targetHeadingForMove(prevRobotX_cm, prevRobotY_cm,
targetRobotX_cm, targetRobotY_cm,
steps[i].reverse);

float turnAngleDeg = fabs(angleDiffDeg(moveHeadingDeg, prevHeadingDeg));
float dx = targetRobotX_cm - prevRobotX_cm;
float dy = targetRobotY_cm - prevRobotY_cm;
float driveDistCm = sqrtf(dx * dx + dy * dy);

totalSec += estimateTurnTimeSecFromAngle(turnAngleDeg);
totalSec += estimateDriveTimeSecFromDistance(driveDistCm);

prevRobotX_cm = targetRobotX_cm;
prevRobotY_cm = targetRobotY_cm;
prevHeadingDeg = moveHeadingDeg;
}

return totalSec;
}

float getElapsedRunTimeSec() {
return (millis() - runStartMs) / 1000.0f;
}

unsigned long computeDelayMsFromUpcomingSteps(const MoveStep* steps, int count) {
if (count <= 0) return 0;

float elapsedSec = getElapsedRunTimeSec();
float remainingRunSec = TARGET_RUN_TIME_SEC - elapsedSec;

if (remainingRunSec <= 0.0f) return 0;

float estRemainingMotionSec = estimateRemainingMotionTimeSec(steps, count);
float extraSlackSec = remainingRunSec - estRemainingMotionSec;

if (extraSlackSec <= 0.0f) return 0;

int remainingWaits = count;
float waitSec = extraSlackSec / remainingWaits;

unsigned long waitMs = (unsigned long)(waitSec * 1000.0f);
if (waitMs > MAX_WAIT_MS) waitMs = MAX_WAIT_MS;

if (DEBUG_SERIAL) {
Serial.print("[WAIT] elapsed=");
Serial.print(elapsedSec, 3);
Serial.print(" remain=");
Serial.print(remainingRunSec, 3);
Serial.print(" estMotion=");
Serial.print(estRemainingMotionSec, 3);
Serial.print(" slack=");
Serial.print(extraSlackSec, 3);
Serial.print(" count=");
Serial.print(count);
Serial.print(" waitMs=");
Serial.println(waitMs);
}

return waitMs;
}

// --------------------------------------------------
// Move wrappers
// --------------------------------------------------
bool Turn(float headingDegTarget) {
return turnToHeadingDeg(headingDegTarget);
}

bool Drive(float distanceCm, float headingDegTarget) {
return driveDistanceCmProfile(distanceCm, headingDegTarget);
}

bool DriveToPointDir(float targetRobotX_cm, float targetRobotY_cm, bool reverse = false) {
float pointHeadingDeg = angleToPointDeg(targetRobotX_cm, targetRobotY_cm);
float driveHeadingDeg = pointHeadingDeg;
float distanceCm = distanceToPointCm(targetRobotX_cm, targetRobotY_cm);

if (reverse) {
driveHeadingDeg = pointHeadingDeg + 180.0f;
driveHeadingDeg = wrapAngleDeg(driveHeadingDeg);
distanceCm = -distanceCm;
}

Serial.print("TURN TO ");
Serial.println(driveHeadingDeg, 2);

if (!Turn(driveHeadingDeg)) {
Serial.println("TURN FAILED");
return false;
}

Serial.print("DRIVE DIST ");
Serial.println(distanceCm, 2);

if (!driveDistanceCmProfile(distanceCm, driveHeadingDeg)) {
Serial.println("DRIVE FAILED");
return false;
}

return true;
}

bool DriveToGrid(float x, float y, bool reverse = false) {
float targetRobotX_cm = gridYToRobotXcm(y);
float targetRobotY_cm = gridXToRobotYcm(x);
return DriveToPointDir(targetRobotX_cm, targetRobotY_cm, reverse);
}

bool runRouteWithAutoDelay(const MoveStep* route, int routeLen) {
for (int i = 0; i < routeLen; i++) {
Serial.print("=== STEP ");
Serial.print(i);
Serial.print(" target=(");
Serial.print(route[i].x, 2);
Serial.print(", ");
Serial.print(route[i].y, 2);
Serial.print(") rev=");
Serial.println(route[i].reverse ? 1 : 0);

if (!DriveToGrid(route[i].x, route[i].y, route[i].reverse)) {
Serial.print("SKIPPING STEP ");
Serial.println(i);
continue;
}

Serial.print("DONE STEP ");
Serial.println(i);

int remaining = routeLen - (i + 1);
if (remaining > 0) {
doPeriodicRecalIfDue();

unsigned long waitMs = computeDelayMsFromUpcomingSteps(&route[i + 1], remaining);
waitMs = capWaitByLastInactive(waitMs);

Serial.print("WAIT AFTER STEP ");
Serial.print(i);
Serial.print(" = ");
Serial.println(waitMs);

doDelayMs(waitMs);
lastInactiveMs += waitMs;
}
}
return true;
}

#define GO(x) do { if (!(x)) return; } while (0)

// --------------------------------------------------
// Run sequence
// --------------------------------------------------
void runCourse() {
calibrateGyroBias();
resetPoseCm(0.0f, -4.0f, 0.0f);
runStartMs = millis();
lastPeriodicRecalMs = millis();

rotev.ledWrite(0, 0, 20);

MoveStep route[] =
{
STEP(0, .5, false),
STEP(0, 1.5, false),
STEP(0, .5, false),
STEP(0, 1.5, false),
STEP(0, .5, false),
STEP(0, 1.5, false),
STEP(0, .5, false),
STEP(0, 0, true),


/*
STEP(0, 2, false),
STEP(-.5, 2.5, false),
STEP(-1, 2, false),
STEP(-1, 1, false),
STEP(-.5, .5, false),
STEP(0, .5, false),
STEP(-1, .5, false),
STEP(0, 1.5, false),
STEP(-1, 2.5, false),
STEP(0, 2.5, false),
STEP(-1, 1.5, false),
STEP(0, .5, false),

STEP(0, .5, false),
STEP(0, 1.5, false),
STEP(0, 2, false),
STEP(-.5, 2.5, false),
STEP(-1, 2, false),
STEP(-1, 1, false),
STEP(-.5, .5, false),
STEP(0, .5, false),
STEP(-1, .5, false),
STEP(0, 1.5, false),
STEP(-1, 2.5, false),
STEP(0, 2.5, false),
STEP(-1, 1.5, false),
STEP(0, .5, false),

STEP(0, .5, false),
STEP(0, 1.5, false),
STEP(0, 2, false),
STEP(-.5, 2.5, false),
STEP(-1, 2, false),
STEP(-1, 1, false),
STEP(-.5, .5, false),
STEP(0, .5, false),
STEP(-1, .5, false),
STEP(0, 1.5, false),
STEP(-1, 2.5, false),
STEP(0, 2.5, false),
STEP(-1, 1.5, false),
STEP(0, .5, false),
*/
};

int routeLen = sizeof(route) / sizeof(route[0]);
GO(runRouteWithAutoDelay(route, routeLen));
GO(Drive(-3.0f, headingRad * RAD_TO_DEG_F));

stopRobot();
rotev.ledWrite(0, 20, 0);

if (DEBUG_SERIAL) {
Serial.print("[FINAL POSE] x_cm=");
Serial.print(posX_cm, 2);
Serial.print(" y_cm=");
Serial.print(posY_cm, 2);
Serial.print(" head=");
Serial.print(headingRad * RAD_TO_DEG_F, 2);
Serial.print(" gx=");
Serial.print(robotYcmToGridX(posY_cm), 2);
Serial.print(" gy=");
Serial.println(robotXcmToGridY(posX_cm), 2);
}
}

// --------------------------------------------------
// Arduino
// --------------------------------------------------
void setup() {
Serial.begin(115200);
rotev.begin();
rotev.motorEnable(true);
stopRobot();

delay(1000);
calibrateGyroBias();

Serial.println("Robot debug ready");
}

void loop() {
stopRobot();

waitForGoRelease();
waitForFreshGoPress();
waitForGoRelease();

runCourse();

stopRobot();

while (rotev.goButtonPressed() || rotev.stopButtonPressed()) {
delay(10);
}
}