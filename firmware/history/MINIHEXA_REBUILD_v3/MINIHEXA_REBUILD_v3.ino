/*
 * MINIHEXA_REBUILD.ino  — v5
 *
 * Uses a custom IK derived from hardware-confirmed geometry and tuned
 * neutral duties. The precompiled kinematics_lib.a is NOT used for
 * femur/tibia — its C-joint output is always out of range. This IK
 * was derived by:
 *   1. Hardware-tuning the neutral stance to spider pose
 *   2. Confirming neutral duties: FemurR=1350, TibiaR=1350 (Left=1750)
 *   3. Back-calculating the physical angles: femur 21° up, tibia 21° outward from down
 *   4. Building 2D IK from the confirmed MiniHexa geometry
 *
 * Geometry (from kinematics.h + 3DM scan, confirmed):
 *   COXA   = 2.85 cm   (hip pivot to femur pivot, horizontal)
 *   FEMUR  = 3.70 cm   (femur bone)
 *   OFFSET = 1.50 cm   (vertical drop at coxa-femur junction)
 *   TIBIA  = 7.20 cm   (tibia bone)
 *
 * Servo conventions (from robot.cpp set_angle()):
 *   dir=true  (right legs, all hips): duty = 500  + 7.142857 * angle
 *   dir=false (left legs femur/tibia): duty = 2500 - 7.142857 * angle
 *   Femur:  angle = MIDPOINT - θ_femur_deg   (MIDPOINT=140)
 *   Tibia:  angle = MIDPOINT - θ_tibia_from_down_deg
 *   Hip:    angle = MIDPOINT + θ_hip_yaw_deg
 *
 * Neutral foot position (local leg frame): x=8.88cm, z=-6.90cm
 *
 * Controls (Bluepad32):
 *   Left stick Y: forward/back
 *   Left stick X: turn left/right
 *   Right stick Y: body height
 *   Cross button: stop and re-centre
 *   Square: toggle IMU levelling
 *   L3: cycle speed (slow/medium/fast)
 */

#include <Bluepad32.h>
#include <math.h>
#include "hiwonder_servo.h"
#include "hiwonder_board.h"

// ----------------------------------------------------------------
// INSTANCES
// ----------------------------------------------------------------
HW_Servo   servo;
HW_Board   board;
GamepadPtr myGamepad = nullptr;

// ----------------------------------------------------------------
// GEOMETRY
// ----------------------------------------------------------------
static const float COXA   = 2.85f;
static const float FEMUR  = 3.70f;
static const float OFFSET = 1.50f;
static const float TIBIA  = 7.20f;

static const float FACTOR = 7.142857f;   // µs per degree (280° servo)
static const float MIDPOINT = 140.0f;

// Neutral foot position in local leg frame (x=outward, z=up)
static const float NEUTRAL_X = 8.884f;
static const float NEUTRAL_Z = -6.896f;

// ----------------------------------------------------------------
// SERVO MAP
// ----------------------------------------------------------------
static const int LEG_COUNT      = 6;
static const int JOINTS_PER_LEG = 3;

const uint8_t SERVO_ID[LEG_COUNT][JOINTS_PER_LEG] = {
  { 1,  2,  3},   // leg 0  right-rear
  { 4,  5,  6},   // leg 1  right-mid
  { 7,  8,  9},   // leg 2  right-front
  {10, 11, 12},   // leg 3  left-front
  {13, 14, 15},   // leg 4  left-mid
  {16, 17, 18}    // leg 5  left-rear
};

// Leg yaw angles: rotation of local x-axis from body x-axis
const float LEG_YAW_DEG[LEG_COUNT] = {
  -45.0f, 0.0f, 45.0f, 135.0f, 180.0f, -135.0f
};

// Body-frame hip pivot positions
static const float BODY_HALF_W = 3.8f;
static const float BODY_HALF_L = 7.0f;
const float LEG_BODY_X[LEG_COUNT] = { BODY_HALF_W,  BODY_HALF_W,  BODY_HALF_W,
                                      -BODY_HALF_W, -BODY_HALF_W, -BODY_HALF_W};
const float LEG_BODY_Y[LEG_COUNT] = {-BODY_HALF_L,  0.0f,  BODY_HALF_L,
                                      BODY_HALF_L,  0.0f, -BODY_HALF_L};

inline bool isRightLeg(int i) { return i <= 2; }
inline bool isTripodA(int i)  { return i==0 || i==2 || i==4; }

// ----------------------------------------------------------------
// CUSTOM IK
// ----------------------------------------------------------------
struct LegDuties { int hip, femur, tibia; };

/*
 * Compute servo duties for one leg given foot position in local leg frame.
 * px = horizontal reach (cm, outward along leg axis)
 * pz = vertical (cm, positive = up, negative = below hip)
 * right = true for right-side legs
 *
 * Returns duties clamped to [500, 2500].
 * On failure (unreachable), returns neutral duties.
 */
LegDuties legIK(float px, float pz, bool right) {
  // Femur pivot is at (COXA, -OFFSET) in local frame
  float rx = px - COXA;
  float rz = pz - (-OFFSET);   // = pz + OFFSET
  float d2 = rx*rx + rz*rz;
  float d  = sqrtf(d2);

  float theta_f_deg, tibia_from_down_deg;

  // Reachability check
  if (d > FEMUR + TIBIA || d < fabsf(FEMUR - TIBIA) || d < 0.001f) {
    // Return neutral on failure
    theta_f_deg       = 21.0f;
    tibia_from_down_deg = 21.0f;
  } else {
    // Femur angle: law of cosines, elbow-up solution
    float alpha    = atan2f(rz, rx);
    float cos_beta = (d2 + FEMUR*FEMUR - TIBIA*TIBIA) / (2.0f * d * FEMUR);
    cos_beta = fmaxf(-1.0f, fminf(1.0f, cos_beta));
    float beta  = acosf(cos_beta);
    float theta_f = alpha + beta;          // elbow-up (femur above line to foot)
    theta_f_deg = theta_f * (180.0f / PI);

    // Tibia absolute angle from horizontal
    float fex = COXA  + FEMUR * cosf(theta_f);
    float fez = -OFFSET + FEMUR * sinf(theta_f);
    float tibia_abs_deg = atan2f(pz - fez, px - fex) * (180.0f / PI);

    // "Outward from straight-down": 0 = tibia points straight down
    tibia_from_down_deg = tibia_abs_deg - (-90.0f);
  }

  // Servo angle conversion:
  // Femur: MIDPOINT - theta_f_deg  (positive theta_f = up = lower angle)
  // Tibia: MIDPOINT - tibia_from_down (positive = outward = lower angle for right)
  float angle_f = MIDPOINT - theta_f_deg;
  float angle_t = MIDPOINT - tibia_from_down_deg;

  // Hip: MIDPOINT for straight out (yaw handled separately)
  float angle_h = MIDPOINT;   // will be overridden by caller for turning

  // Apply direction (right=true: duty=500+FACTOR*angle, left: duty=2500-FACTOR*angle)
  auto duty = [&](float angle, bool dir) -> int {
    angle = fmaxf(0.0f, fminf(280.0f, angle));
    return dir ? (int)(500.0f + FACTOR * angle)
               : (int)(2500.0f - FACTOR * angle);
  };

  return {
    duty(angle_h, true),          // hip always dir=true
    duty(angle_f, right),         // femur
    duty(angle_t, right)          // tibia
  };
}

// Clamp duty to safe servo range
inline uint16_t clampDuty(int v) {
  return (uint16_t)(v < 500 ? 500 : v > 2500 ? 2500 : v);
}

// ----------------------------------------------------------------
// GAIT PARAMETERS
// ----------------------------------------------------------------
struct SpeedProfile { float rateHz, strideCm, liftCm; uint16_t moveMs; };
const SpeedProfile SPEEDS[3] = {
  {0.55f, 1.8f, 2.0f, 90},   // slow
  {0.75f, 2.5f, 2.8f, 65},   // medium
  {0.95f, 3.2f, 3.2f, 52},   // fast
};
int speedIdx = 1;

// ----------------------------------------------------------------
// BODY CONTROL
// ----------------------------------------------------------------
static const float BODY_HEIGHT_MAX_CM = 2.0f;   // cm up/down from neutral
static const float BODY_ROLL_MAX_DEG  = 12.0f;
static const float BODY_PITCH_MAX_DEG = 12.0f;

// ----------------------------------------------------------------
// STATE
// ----------------------------------------------------------------
float walkPhase   = 0.0f;
unsigned long lastMs = 0;

float fwdCmd=0, fwdF=0, turnCmd=0, turnF=0;
float heightCmd=0, heightF=0;
bool walkActive = false, walkEnabled = true;

float imuRollZero=0, imuPitchZero=0;
float imuRollFilt=0, imuPitchFilt=0;
float imuRollCmd=0,  imuPitchCmd=0;
bool  imuEnabled=true, imuZeroed=false;

static const float IMU_FILTER  = 0.10f;
static const float IMU_GAIN    = 0.07f;
static const float IMU_MAX_CM  = 1.5f;
static const float WALK_SCALE  = 0.30f;

// ----------------------------------------------------------------
// HELPERS
// ----------------------------------------------------------------
inline float clampf(float v, float lo, float hi){ return v<lo?lo:v>hi?hi:v; }
inline int   clampi(int v, int lo, int hi)      { return v<lo?lo:v>hi?hi:v; }
inline int   applyDB(int v, int db)             { return (v>-db&&v<db)?0:v; }

// ----------------------------------------------------------------
// CONTROLLER CALLBACKS
// ----------------------------------------------------------------
void onConnectedGamepad(GamepadPtr gp) {
  Serial.println("Controller connected");
  myGamepad = gp;
}
void onDisconnectedGamepad(GamepadPtr gp) {
  Serial.println("Controller disconnected");
  if (myGamepad == gp) myGamepad = nullptr;
}

void captureImuZero() {
  float e[3]={0,0,0};
  board.get_imu_euler(e);
  imuRollZero = e[0]; imuPitchZero = e[1];
  imuRollFilt = imuPitchFilt = imuRollCmd = imuPitchCmd = 0;
  imuZeroed = true;
}

// ----------------------------------------------------------------
// INPUT PROCESSING
// ----------------------------------------------------------------
void processInput() {
  if (!myGamepad || !myGamepad->isConnected()) return;

  static bool lastCross=false, lastSquare=false, lastL3=false;

  int lx = applyDB(myGamepad->axisX(),  50);
  int ly = applyDB(myGamepad->axisY(),  50);
  int ry = applyDB(myGamepad->axisRY(), 50);

  float rawFwd  = clampf(-(float)ly/512.f,-1,1);
  float rawTurn = clampf( (float)lx/512.f,-1,1);
  float rawHt   = clampf(-(float)ry/512.f,-1,1);

  fwdF    = 0.82f*fwdF    + 0.18f*rawFwd;
  turnF   = 0.82f*turnF   + 0.18f*rawTurn;
  heightF = 0.85f*heightF + 0.15f*rawHt;

  fwdCmd   = clampf(fwdF,   -1,1);
  turnCmd  = clampf(turnF,  -1,1);
  heightCmd= clampf(heightF,-1,1);

  float mag = fmaxf(fabsf(fwdCmd), fabsf(turnCmd));
  walkActive = walkEnabled && mag >= 0.12f;

  bool cross  = myGamepad->a();
  bool square = myGamepad->x();
  bool l3     = myGamepad->thumbL();

  if (cross && !lastCross) {
    walkPhase=0; lastMs=0;
    fwdCmd=fwdF=turnCmd=turnF=0;
    heightCmd=heightF=0;
    walkActive=false;
    captureImuZero();
  }
  if (square && !lastSquare) {
    imuEnabled = !imuEnabled;
    imuRollCmd = imuPitchCmd = 0;
    if (imuEnabled) captureImuZero();
  }
  if (l3 && !lastL3) {
    speedIdx = (speedIdx + 1) % 3;
    Serial.printf("Speed: %s\n", speedIdx==0?"SLOW":speedIdx==1?"MEDIUM":"FAST");
  }

  lastCross=cross; lastSquare=square; lastL3=l3;
}

// ----------------------------------------------------------------
// IMU LEVELLING
// ----------------------------------------------------------------
void updateIMU() {
  if (!imuEnabled || !imuZeroed) { imuRollCmd=imuPitchCmd=0; return; }

  float e[3]={0,0,0};
  board.get_imu_euler(e);

  float re = e[0]-imuRollZero, pe = e[1]-imuPitchZero;
  imuRollFilt  = (1-IMU_FILTER)*imuRollFilt  + IMU_FILTER*re;
  imuPitchFilt = (1-IMU_FILTER)*imuPitchFilt + IMU_FILTER*pe;

  if (fabsf(imuRollFilt)  < 0.8f) imuRollFilt  = 0;
  if (fabsf(imuPitchFilt) < 1.0f) imuPitchFilt = 0;

  float sc = walkActive ? WALK_SCALE : 1.0f;
  imuRollCmd  = clampf(-imuRollFilt  * IMU_GAIN * sc, -IMU_MAX_CM, IMU_MAX_CM);
  imuPitchCmd = clampf(-imuPitchFilt * IMU_GAIN * sc, -IMU_MAX_CM, IMU_MAX_CM);
}

// ----------------------------------------------------------------
// GAIT PHASE
// ----------------------------------------------------------------
void updatePhase() {
  unsigned long now = millis();
  if (!lastMs) { lastMs=now; return; }
  float dt = (now-lastMs)*0.001f;
  lastMs = now;
  if (!walkActive) return;
  float mag = clampf(fmaxf(fabsf(fwdCmd),fabsf(turnCmd)),0,1);
  walkPhase += dt * SPEEDS[speedIdx].rateHz * clampf(0.5f+0.5f*mag, 0.3f, 1.1f);
  while (walkPhase >= 1.0f) walkPhase -= 1.0f;
}

// ----------------------------------------------------------------
// SERVO OUTPUT
// ----------------------------------------------------------------
void writeServos() {
  const SpeedProfile& sp = SPEEDS[speedIdx];
  float sinP = sinf(2.0f * PI * walkPhase);
  float cosP = cosf(2.0f * PI * walkPhase);

  float htCm     = heightCmd * BODY_HEIGHT_MAX_CM;
  float cmdMag   = clampf(fabsf(fwdCmd) + fabsf(turnCmd)*0.6f, 0.0f, 1.0f);
  float strideAmp = sp.strideCm * cmdMag;
  float liftAmp   = sp.liftCm   * clampf(cmdMag, 0.3f, 1.0f);

  ServoArg_t s[LEG_COUNT * JOINTS_PER_LEG];
  int idx = 0;

  for (int i = 0; i < LEG_COUNT; i++) {
    bool right = isRightLeg(i);
    bool tA    = isTripodA(i);

    // Tripod phase: A and B groups alternate stride/swing
    float strideSin = tA ?  sinP : -sinP;
    float liftCos   = tA ?  cosP : -cosP;   // positive = this group in swing

    // ---- STRIDE: compute delta in BODY frame then rotate to local leg frame ----
    //
    // Body frame convention: +x = robot right, +y = robot forward
    // Foot sweeps BACKWARD relative to body during stance (body moves forward).
    // So for fwdCmd>0 (move forward): foot body-delta = (0, -stride*sin)
    // For turnCmd>0 (turn CW/right): foot tangential body-delta = (by, -bx)*turn*sin/r

    // Forward component in body frame
    float bd_x = 0.0f;
    float bd_y = -fwdCmd * strideAmp * strideSin;

    // Turn component: tangential velocity for CW rotation at this leg's body position
    float bx = LEG_BODY_X[i], by = LEG_BODY_Y[i];
    float r = sqrtf(bx*bx + by*by);
    if (r > 0.001f) {
      // CW rotation: foot moves CCW → tangential direction = (by, -bx)/r
      bd_x += turnCmd * strideAmp * (by / r) * strideSin;
      bd_y += turnCmd * strideAmp * (-bx / r) * strideSin;
    }

    // Rotate body-frame delta into local leg frame (rotate by -yaw)
    float yawRad = LEG_YAW_DEG[i] * (PI / 180.0f);
    float ld_x =  cosf(yawRad)*bd_x + sinf(yawRad)*bd_y;
    float ld_y = -sinf(yawRad)*bd_x + cosf(yawRad)*bd_y;

    // ---- FOOT POSITION in local leg frame ----
    float footX = NEUTRAL_X + ld_x;
    float footY =              ld_y;   // lateral offset (drives hip yaw)
    float footZ = NEUTRAL_Z - htCm;   // height: negative Z = lower foot = raise body

    // IMU levelling: z offset per leg based on body-frame position
    float tiltZ = by * sinf(imuPitchCmd * (PI/180.0f))
                - bx * sinf(imuRollCmd  * (PI/180.0f));
    footZ -= tiltZ;

    // Lift during swing
    if (walkActive && liftCos > 0.0f) {
      footZ += liftCos * liftAmp;
    }

    // ---- IK for femur and tibia ----
    LegDuties d = legIK(footX, footZ, right);

    // ---- HIP YAW from footY ----
    // Hip servo yaws the leg around the vertical axis.
    // footY > 0 means foot is displaced forward of the leg axis (for yaw=0 leg).
    // Hip yaw angle = atan2(footY, footX) in degrees.
    // Servo: angle = MIDPOINT + yaw_delta (dir=true for all hips)
    float hipYawDeg = atan2f(footY, footX) * (180.0f / PI);
    hipYawDeg = clampf(hipYawDeg, -25.0f, 25.0f);
    float hipAngle = MIDPOINT + hipYawDeg;
    d.hip = (int)(500.0f + FACTOR * hipAngle);

    s[idx++] = {SERVO_ID[i][0], clampDuty(d.hip)};
    s[idx++] = {SERVO_ID[i][1], clampDuty(d.femur)};
    s[idx++] = {SERVO_ID[i][2], clampDuty(d.tibia)};
  }

  uint16_t moveMs = walkActive ? SPEEDS[speedIdx].moveMs : 100;
  servo.multi_set(s, LEG_COUNT * JOINTS_PER_LEG, moveMs);
}

// ----------------------------------------------------------------
// STARTUP RAMP
// ----------------------------------------------------------------
void rampToNeutral() {
  LegDuties neutral[LEG_COUNT];
  for (int i = 0; i < LEG_COUNT; i++)
    neutral[i] = legIK(NEUTRAL_X, NEUTRAL_Z, isRightLeg(i));

  for (int step = 1; step <= 60; step++) {
    float t = (float)step / 60.0f;
    ServoArg_t s[LEG_COUNT * JOINTS_PER_LEG];
    int idx = 0;
    for (int i = 0; i < LEG_COUNT; i++) {
      auto interp = [&](int target) {
        return clampDuty((int)(1500.0f + t * (target - 1500)));
      };
      s[idx++] = {SERVO_ID[i][0], interp(neutral[i].hip)};
      s[idx++] = {SERVO_ID[i][1], interp(neutral[i].femur)};
      s[idx++] = {SERVO_ID[i][2], interp(neutral[i].tibia)};
    }
    servo.multi_set(s, LEG_COUNT * JOINTS_PER_LEG, 55);
    delay(28);
  }
}

// ----------------------------------------------------------------
// SETUP / LOOP
// ----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  servo.begin();
  board.begin();
  board.imu_update(true);
  delay(800);

  // Verify IK round-trip
  LegDuties nd = legIK(NEUTRAL_X, NEUTRAL_Z, true);
  Serial.printf("MiniHexa v5 — custom IK\n");
  Serial.printf("Neutral IK check (right leg): femur=%d tibia=%d (expect 1350/1350)\n",
                nd.femur, nd.tibia);

  rampToNeutral();
  delay(300);
  captureImuZero();

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  Serial.println("Ready.");
}

void loop() {
  BP32.update();
  processInput();
  updateIMU();
  updatePhase();
  writeServos();
  delay(10);
}
