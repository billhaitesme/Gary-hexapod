/*
 * MINIHEXA_DIAG.ino
 *
 * PUT THIS IN ITS OWN FOLDER: Documents/Arduino/MINIHEXA_DIAG/
 * Copy into that folder ONLY these files from your MINIHEXA_REBUILD folder:
 *   - kinematics.h
 *   - kinematics_lib.a
 *
 * Do NOT copy global.h, hiwonder_servo.h, hiwonder_robot.cpp, etc.
 * This sketch is fully self-contained and needs nothing else.
 *
 * No servos move. Flash, open Serial Monitor at 115200, paste the output.
 */

#include <Arduino.h>
#include <math.h>
#include "kinematics.h"

// ----------------------------------------------------------------
// Local copy of the duty formula — no other headers needed
// ----------------------------------------------------------------
static const float KNOT_MID = 140.0f;
static const float F280     = 7.142857f;

static int dutyFwd(float angle) {   // dir = true  (right side)
  if (angle <   0.0f) angle =   0.0f;
  if (angle > 280.0f) angle = 280.0f;
  return (int)(500.0f + F280 * angle);
}
static int dutyRev(float angle) {   // dir = false (left side)
  if (angle <   0.0f) angle =   0.0f;
  if (angle > 280.0f) angle = 280.0f;
  return (int)(2500.0f - F280 * angle);
}

// For a right-side leg (dir=true):
//   hip  duty = dutyFwd(KNOT_MID + a)
//   fem  duty = dutyFwd(KNOT_MID - b)
//   tib  duty = dutyFwd(KNOT_MID + c)
static int hipDuty(float a) { return dutyFwd(KNOT_MID + a); }
static int femDuty(float b) { return dutyFwd(KNOT_MID - b); }
static int tibDuty(float c) { return dutyFwd(KNOT_MID + c); }

static bool safe(int d) { return d >= 900 && d <= 2100; }

// ----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println(F("\n\n=== MINIHEXA IK/FK DIAGNOSTIC ===\n"));

  // ----------------------------------------------------------------
  // SWEEP A — raw ikine() over a grid of (x, z) values, y=0
  // Shows which foot positions give safe duty on both femur and tibia
  // ----------------------------------------------------------------
  Serial.println(F("--- SWEEP A: ikine(x, 0, z) ---"));
  Serial.println(F("Columns: status | x | z | a | b | c | femDuty | tibDuty"));

  const float xs[] = {3,4,5,6,6.5,7,8,9,10,11};
  const float zs[] = {-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6};
  const int NX = 10, NZ = 13;

  for (int zi = 0; zi < NZ; zi++) {
    for (int xi = 0; xi < NX; xi++) {
      Vector_t pt = {xs[xi], 0.0f, zs[zi]};
      Theta_t  th = ikine(&pt);
      int fd = femDuty(th.b);
      int td = tibDuty(th.c);
      bool ok = safe(fd) && safe(td);
      Serial.printf("%s  x=%5.1f z=%5.1f | a=%8.3f b=%8.3f c=%8.3f | fem=%4d tib=%4d\n",
                    ok ? "[OK ]" : "[bad]",
                    xs[xi], zs[zi],
                    th.a, th.b, th.c, fd, td);
    }
  }

  // ----------------------------------------------------------------
  // SWEEP B — fkine() with positive b and c (standard standing guess)
  // Shows where the foot ends up for sensible joint angles
  // ----------------------------------------------------------------
  Serial.println(F("\n--- SWEEP B: fkine(a=0, b, c) positive c ---"));
  Serial.println(F("Columns: b | c | foot_x | foot_y | foot_z"));

  const float bs[] = {0,10,20,30,40,50,60,70,80};
  const float cs_pos[] = {0,10,20,30,40,50,60,70,80,90,100,110,120,130};
  const int NB = 9, NCP = 14;

  for (int bi = 0; bi < NB; bi++) {
    for (int ci = 0; ci < NCP; ci++) {
      Theta_t theta = {0.0f, bs[bi], cs_pos[ci]};
      Vector_t foot = fkine(&theta);
      Serial.printf("b=%5.1f c=%6.1f -> (%8.4f, %8.4f, %8.4f)\n",
                    bs[bi], cs_pos[ci], foot.x, foot.y, foot.z);
    }
    Serial.println();
  }

  // ----------------------------------------------------------------
  // SWEEP C — fkine() with NEGATIVE c values
  // In case the library's standing pose uses negative c
  // Only prints rows where the foot looks like a plausible standing position
  // (x > 2, -15 < z < 1, |y| < 0.5)
  // ----------------------------------------------------------------
  Serial.println(F("\n--- SWEEP C: fkine(a=0, b, c) NEGATIVE c --- (filtered to plausible standing)"));

  const float cs_neg[] = {-10,-20,-30,-40,-50,-60,-70,-80,
                           -90,-100,-110,-120,-130,-140,-150};
  const int NCN = 15;

  for (int bi = 0; bi < NB; bi++) {
    for (int ci = 0; ci < NCN; ci++) {
      Theta_t theta = {0.0f, bs[bi], cs_neg[ci]};
      Vector_t foot = fkine(&theta);
      if (foot.x > 2.0f && foot.z < 1.0f && foot.z > -15.0f
          && fabsf(foot.y) < 0.5f) {
        Serial.printf("b=%5.1f c=%7.1f -> (%8.4f, %8.4f, %8.4f)  <standing candidate>\n",
                      bs[bi], cs_neg[ci], foot.x, foot.y, foot.z);
      }
    }
  }

  // ----------------------------------------------------------------
  // SWEEP D — round-trip: fkine then ikine, checking consistency
  // Feeds plausible (b, c) pairs into fkine, then feeds that foot
  // position back into ikine, and shows whether we get the same angles back.
  // This reveals how consistent the library is internally.
  // ----------------------------------------------------------------
  Serial.println(F("\n--- SWEEP D: round-trip fkine->ikine for standing-looking combos ---"));
  Serial.println(F("Format: b_in c_in -> foot -> b_out c_out [match?]"));

  const float rt_bs[] = {20,30,40,50};
  const float rt_cs[] = {20,30,40,50,60,70,80,90,100};
  const int NRB = 4, NRC = 9;

  for (int bi = 0; bi < NRB; bi++) {
    for (int ci = 0; ci < NRC; ci++) {
      Theta_t in_theta = {0.0f, rt_bs[bi], rt_cs[ci]};
      Vector_t foot    = fkine(&in_theta);
      Theta_t out_theta= ikine(&foot);

      bool b_match = fabsf(out_theta.b - rt_bs[bi]) < 0.1f;
      bool c_match = fabsf(out_theta.c - rt_cs[ci]) < 0.1f;

      Serial.printf("b=%4.0f c=%4.0f -> foot(%7.3f,%7.3f,%7.3f) -> b=%8.3f c=%8.3f [b:%s c:%s]\n",
                    rt_bs[bi], rt_cs[ci],
                    foot.x, foot.y, foot.z,
                    out_theta.b, out_theta.c,
                    b_match ? "OK" : "NO", c_match ? "OK" : "NO");
    }
    Serial.println();
  }

  // ----------------------------------------------------------------
  // SWEEP E — vendor reference points, tested directly
  // Shows exactly what the library produces for the known poses
  // ----------------------------------------------------------------
  Serial.println(F("\n--- SWEEP E: vendor reference points ---"));

  struct RefPt { const char* name; float x,y,z; };
  const RefPt refs[] = {
    {"begin_point   {6.5,0,-4}", 6.5f, 0.0f, -4.0f},
    {"verified_point{9.0,0,-2}", 9.0f, 0.0f, -2.0f},
    {"{6.5, 0, -3}            ", 6.5f, 0.0f, -3.0f},
    {"{6.5, 0, -2}            ", 6.5f, 0.0f, -2.0f},
    {"{8.0, 0, -2}            ", 8.0f, 0.0f, -2.0f},
    {"{8.0, 0, -3}            ", 8.0f, 0.0f, -3.0f},
    {"{9.0, 0, -3}            ", 9.0f, 0.0f, -3.0f},
    {"{10.0,0, -2}            ",10.0f, 0.0f, -2.0f},
    {"{6.5, 0,  4}            ", 6.5f, 0.0f,  4.0f},
    {"{6.5, 0,  2}            ", 6.5f, 0.0f,  2.0f},
  };
  const int NREF = 10;

  for (int i = 0; i < NREF; i++) {
    Vector_t pt   = {refs[i].x, refs[i].y, refs[i].z};
    Theta_t  th   = ikine(&pt);
    Vector_t rtfk = fkine(&th);   // round-trip

    int fd = femDuty(th.b);
    int td = tibDuty(th.c);
    bool ok = safe(fd) && safe(td);

    Serial.printf("\n%s  %s\n", ok ? "[OK ]" : "[bad]", refs[i].name);
    Serial.printf("  in:      (%.3f, %.3f, %.3f)\n", refs[i].x, refs[i].y, refs[i].z);
    Serial.printf("  theta:   a=%.4f  b=%.4f  c=%.4f\n", th.a, th.b, th.c);
    Serial.printf("  fk-back: (%.3f, %.3f, %.3f)\n", rtfk.x, rtfk.y, rtfk.z);
    Serial.printf("  duties:  hip=%d  fem=%d  tib=%d\n",
                  hipDuty(th.a), fd, td);
  }

  Serial.println(F("\n=== DIAGNOSTIC COMPLETE ==="));
  Serial.println(F("Copy everything above and paste it back."));
}

void loop() {
  delay(60000);
}
