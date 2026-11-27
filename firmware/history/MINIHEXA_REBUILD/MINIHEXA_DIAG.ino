/*
 * MINIHEXA_DIAG.ino
 *
 * DIAGNOSTIC ONLY — no servos move.
 * Flash this, open Serial Monitor at 115200, paste the full output.
 *
 * Runs two sweeps:
 *   1. IK sweep  — feeds ikine() a grid of (x,z) foot positions,
 *                  prints which ones produce safe servo duties.
 *   2. FK sweep  — feeds fkine() a grid of (b,c) joint angles,
 *                  prints where the foot lands.
 *
 * These two sweeps together reveal the exact coordinate convention
 * of the compiled kinematics library so we can set LEG_START_POINT
 * to a position that is actually inside the IK workspace.
 */

#include <math.h>
#include "hiwonder_servo.h"
#include "hiwonder_board.h"
#include "kinematics.h"

// ----------------------------------------------------------------
// CONSTANTS  (must match the final firmware exactly)
// ----------------------------------------------------------------
static const float KNOT_MIDPOINT = 140.0f;
static const float FACTOR_280    = 7.142857f;

static const float BODY_HALF_W   = 3.8f;
static const float BODY_HALF_L   = 7.0f;

static const int   LEG_COUNT     = 6;

const Vector_t LEG_BODY_START[LEG_COUNT] = {
  { BODY_HALF_W, -BODY_HALF_L, 0.0f},
  { BODY_HALF_W,  0.0f,        0.0f},
  { BODY_HALF_W,  BODY_HALF_L, 0.0f},
  {-BODY_HALF_W,  BODY_HALF_L, 0.0f},
  {-BODY_HALF_W,  0.0f,        0.0f},
  {-BODY_HALF_W, -BODY_HALF_L, 0.0f}
};

const float LEG_YAW_DEG[LEG_COUNT] = {
  -45.0f, 0.0f, 45.0f, 135.0f, 180.0f, -135.0f
};

// Direction flags  (right-side legs use dir=true, left-side false)
const bool HIP_DIR[LEG_COUNT] = {true,  true,  true,  true,  true,  true };
const bool B_DIR[LEG_COUNT]   = {true,  true,  true,  false, false, false};
const bool C_DIR[LEG_COUNT]   = {true,  true,  true,  false, false, false};

// ----------------------------------------------------------------
// HELPERS
// ----------------------------------------------------------------
inline int angleToDuty(bool dir, float angle) {
  if (angle <   0.0f) angle =   0.0f;
  if (angle > 280.0f) angle = 280.0f;
  if (dir) return (int)(500.0f  + FACTOR_280 * angle);
  else     return (int)(2500.0f - FACTOR_280 * angle);
}

// ----------------------------------------------------------------
// SETUP  — all work happens here, loop() does nothing
// ----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);   // give Serial Monitor time to connect

  Serial.println("\n\n========================================");
  Serial.println(" MINIHEXA IK/FK DIAGNOSTIC");
  Serial.println("========================================\n");

  // ------------------------------------------------------------------
  // SWEEP 1 — pose_control() output for each leg at zero body pose
  // Shows what the library computes for the neutral foot position
  // before we even call ikine().
  // ------------------------------------------------------------------
  Serial.println("--- SWEEP 1: pose_control neutral results ---");
  Serial.println("(leg, yaw_deg, b_leg_start, r_leg_end, neutral_result)");

  Vector_t pos_zero = {0.0f, 0.0f, 0.0f};
  Euler_t  eul_zero = {0.0f, 0.0f, 0.0f};

  // Candidate neutral foot points to test (in local leg frame)
  float cand_x[] = {6.5f, 7.0f, 8.0f, 9.0f};
  float cand_z[] = {-4.0f, -3.0f, -2.0f, -1.0f};

  for (int ci = 0; ci < 4; ci++) {
    for (int cj = 0; cj < 4; cj++) {
      Vector_t start_local = {cand_x[ci], 0.0f, cand_z[cj]};

      // Use leg 0 geometry (right-rear, yaw=-45°)
      Vector_t r_end = rotation_trans(&start_local, LEG_YAW_DEG[0], AXIS_Z);
      Vector_t b_end = vector_arg_ops(&r_end, (Vector_t*)&LEG_BODY_START[0], ADD);
      Vector_t result= pose_control(1, &pos_zero, &eul_zero,
                                    &r_end, (Vector_t*)&LEG_BODY_START[0]);

      Theta_t th = ikine(&result);

      int hipD = angleToDuty(HIP_DIR[0], KNOT_MIDPOINT + th.a);
      int bD   = angleToDuty(B_DIR[0],   KNOT_MIDPOINT - th.b);
      int cD   = angleToDuty(C_DIR[0],   KNOT_MIDPOINT + th.c);

      bool safe = (bD > 800 && bD < 2200 && cD > 800 && cD < 2200);

      Serial.printf("%s  local=(%.1f,0,%.1f)  result=(%.3f,%.3f,%.3f)"
                    "  theta=(a=%.2f b=%.2f c=%.2f)"
                    "  duty=(hip=%d b=%d c=%d)\n",
                    safe ? "[SAFE]" : "[BAD ]",
                    cand_x[ci], cand_z[cj],
                    result.x, result.y, result.z,
                    th.a, th.b, th.c,
                    hipD, bD, cD);
    }
  }

  // ------------------------------------------------------------------
  // SWEEP 2 — raw ikine() over a grid of (x, z) foot positions
  // No pose_control, no body frame — feeds the library directly.
  // This shows what foot coordinates give sane C-joint duty.
  // ------------------------------------------------------------------
  Serial.println("\n--- SWEEP 2: raw ikine(x, 0, z) ---");
  Serial.println("Format: [SAFE/bad] x=.. z=..  a=.. b=.. c=..  bDuty=.. cDuty=..");

  float xs[] = {3.0f, 4.0f, 5.0f, 6.0f, 6.5f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f};
  float zs[] = {-6.0f, -5.0f, -4.0f, -3.0f, -2.0f, -1.0f,
                  0.0f,  1.0f,  2.0f,  3.0f,  4.0f,  5.0f, 6.0f};

  int good_count = 0;
  for (float tz : zs) {
    for (float tx : xs) {
      Vector_t pt = {tx, 0.0f, tz};
      Theta_t th  = ikine(&pt);

      int bD = angleToDuty(B_DIR[0],   KNOT_MIDPOINT - th.b);
      int cD = angleToDuty(C_DIR[0],   KNOT_MIDPOINT + th.c);

      // Safe = both duties well inside servo travel
      bool safe = (bD > 900 && bD < 2100 && cD > 900 && cD < 2100);
      if (safe) good_count++;

      Serial.printf("%s  x=%5.1f z=%5.1f  |  a=%7.2f b=%7.2f c=%7.2f  |  bDuty=%4d cDuty=%4d\n",
                    safe ? "[SAFE]" : "[ bad]",
                    tx, tz,
                    th.a, th.b, th.c,
                    bD, cD);
    }
  }
  Serial.printf("\n%d safe positions found in sweep 2.\n", good_count);

  // ------------------------------------------------------------------
  // SWEEP 3 — fkine() grid over (b, c) joint angles
  // Shows where the foot lands for plausible standing joint angles.
  // This is the ground truth for what the library considers "reachable".
  // ------------------------------------------------------------------
  Serial.println("\n--- SWEEP 3: fkine(a=0, b, c) ---");
  Serial.println("Format: b=.. c=..  -> foot=(x, y, z)");

  float bs[] = {0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f, 80.0f};
  float cs[] = {0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f,
                70.0f, 80.0f, 90.0f, 100.0f, 110.0f, 120.0f};

  for (float tb : bs) {
    for (float tc : cs) {
      Theta_t theta = {0.0f, tb, tc};
      Vector_t foot = fkine(&theta);
      Serial.printf("b=%5.1f c=%6.1f  ->  (x=%7.3f, y=%7.3f, z=%7.3f)\n",
                    tb, tc, foot.x, foot.y, foot.z);
    }
    Serial.println();
  }

  // ------------------------------------------------------------------
  // SWEEP 4 — same fkine but with NEGATIVE c values
  // (in case the library uses c < 0 for a standing pose)
  // ------------------------------------------------------------------
  Serial.println("\n--- SWEEP 4: fkine(a=0, b, c) with NEGATIVE c ---");
  float neg_cs[] = {-10.0f, -20.0f, -30.0f, -40.0f, -50.0f,
                    -60.0f, -70.0f, -80.0f, -90.0f, -100.0f,
                    -110.0f, -120.0f, -130.0f, -140.0f};

  for (float tb : bs) {
    for (float tc : neg_cs) {
      Theta_t theta = {0.0f, tb, tc};
      Vector_t foot = fkine(&theta);
      // Only print if foot position looks like a standing leg
      // (x > 2, z between -10 and 0, y near 0)
      if (foot.x > 2.0f && foot.z < 0.5f && foot.z > -15.0f
          && fabsf(foot.y) < 1.0f) {
        Serial.printf("b=%5.1f c=%7.1f  ->  (x=%7.3f, y=%7.3f, z=%7.3f)  <-- standing candidate\n",
                      tb, tc, foot.x, foot.y, foot.z);
      }
    }
  }

  // ------------------------------------------------------------------
  // SWEEP 5 — the vendor's own begin_point and verified_point,
  // plus the raw ikine output so we can see all angle components.
  // ------------------------------------------------------------------
  Serial.println("\n--- SWEEP 5: vendor reference points ---");

  struct { const char* name; float x, y, z; } ref[] = {
    {"begin_point   ", 6.5f,  0.0f, -4.0f},
    {"verified_point", 9.0f,  0.0f, -2.0f},
    {"begin_z-3     ", 6.5f,  0.0f, -3.0f},
    {"begin_z-2     ", 6.5f,  0.0f, -2.0f},
    {"x8_z-2        ", 8.0f,  0.0f, -2.0f},
    {"x9_z-3        ", 9.0f,  0.0f, -3.0f},
    {"x10_z-2       ", 10.0f, 0.0f, -2.0f},
  };

  for (auto& r : ref) {
    Vector_t pt = {r.x, r.y, r.z};
    Theta_t th  = ikine(&pt);
    Vector_t fk = fkine(&th);           // round-trip check

    int bD = angleToDuty(B_DIR[0],   KNOT_MIDPOINT - th.b);
    int cD = angleToDuty(C_DIR[0],   KNOT_MIDPOINT + th.c);
    bool safe = (bD > 900 && bD < 2100 && cD > 900 && cD < 2100);

    Serial.printf("%s  %s\n", safe ? "[SAFE]" : "[ bad]", r.name);
    Serial.printf("  ikine input : (%.3f, %.3f, %.3f)\n", r.x, r.y, r.z);
    Serial.printf("  theta       : a=%.4f  b=%.4f  c=%.4f\n", th.a, th.b, th.c);
    Serial.printf("  fkine check : (%.3f, %.3f, %.3f)\n", fk.x, fk.y, fk.z);
    Serial.printf("  duty        : hip=%d  b=%d  c=%d\n\n", 
                  angleToDuty(HIP_DIR[0], KNOT_MIDPOINT + th.a), bD, cD);
  }

  Serial.println("========================================");
  Serial.println(" DIAGNOSTIC COMPLETE — paste above output");
  Serial.println("========================================\n");
}

// ----------------------------------------------------------------
// LOOP — nothing, all work done in setup()
// ----------------------------------------------------------------
void loop() {
  // intentionally empty
  delay(10000);
}
