// gary_geometry.h — Gary's leg/body geometry measured from Hiwonder's own build model
// (minihexa3 003.3dm, Rhino 8, mm; 947 parts, every servo/bracket/blade individually modeled).
// Extracted 2026-08-18 by cad_tools/dump_parts.py + leg_geometry.py + foot_tip.py.
// All six legs came out identical to 0.01 mm, so these are single numbers, not per-leg tables.
//
// NOT YET USED BY THE FIRMWARE. The live IK is Hiwonder's precompiled kinematics_lib.a, whose
// constants (kinematics.h) are listed alongside for comparison. Include this when we (a) add
// pose/torque limits on top of the vendor IK, or (b) replace the IK. Units: mm and degrees
// (the vendor lib uses cm — convert at the boundary).
#pragma once

// ---------------------------------------------------------------------------------------------
// Link lengths — pose-invariant, measured as servo-axis-to-servo-axis distances
// ---------------------------------------------------------------------------------------------
//                                              CAD (mm)   vendor kinematics.h        delta
#define GEO_L1_COXA_MM        29.20f    //  coxa axis -> femur axis (horizontal)   BODY_TOP_LEN   28.5   +0.7
#define GEO_L2_FEMUR_MM       37.83f    //  femur axis -> tibia (knee) axis        TOP_MIDDLE_LEN 37.0   +0.8
#define GEO_L3_TIBIA_MM       74.35f    //  knee axis -> centre of foot-pad bottom  MIDDLE_BASE_LEN 72.0  +2.4
#define GEO_L3_BLADE_MAX_MM   74.10f    //  knee axis -> farthest point of the sheet-metal tibia blade
// Femur axis and knee axis are exactly parallel (0.0 deg); coxa axis is exactly perpendicular to both.
// The femur axis sits 4.08 mm BELOW the coxa servo's horn face and 1.6 mm above the main chassis plate.
#define GEO_FEMUR_AXIS_BELOW_COXA_HUB_MM  4.08f
// The foot pad is in the leg's mid-plane (no lateral offset). Vendor JOINT_POS_OFFSET 1.5 cm has no
// obvious counterpart in the CAD; the precompiled lib's use of it is unknown (see notes/CAD-GEOMETRY-2026-08-18.md).

// ---------------------------------------------------------------------------------------------
// Body — coxa axis positions in the body frame (origin = body centre, +x = right/leg2 side, z up)
// Vendor LEGn_START_X/Y (cm) in hiwonder_robot.h agree to <0.5 mm.
// ---------------------------------------------------------------------------------------------
#define GEO_COXA_MID_RADIUS_MM    43.55f   // middle legs (vendor 43.4)     at azimuth   0 / 180 deg
#define GEO_COXA_CORNER_X_MM      31.32f   // front/rear legs (vendor 31.5)
#define GEO_COXA_CORNER_Y_MM      57.00f   //                 (vendor 56.5)  -> radius 65.0, azimuth +-61.2 deg
#define GEO_COXA_CORNER_AZ_DEG    61.2f    // vendor IK rotates corner legs by +-45 deg — that is a *frame* choice, not the mount angle
#define GEO_MAIN_PLATE_Z_MM       52.7f    // 2 mm main chassis plate (107 x 141), z of its centre in the model
#define GEO_LOWER_PLATE_Z_MM      25.9f    // 1.5 mm lower plate  (77 x 123)  -> ~17 mm ground clearance in the CAD pose
#define GEO_TOP_PLATE_Z_MM        79.4f    // 1.5 mm top plate    (77 x 131)

// ---------------------------------------------------------------------------------------------
// The pose the model is drawn in (a real standing pose — useful as a reference/home pose)
// ---------------------------------------------------------------------------------------------
#define GEO_POSE_FEMUR_PITCH_DEG      45.0f   // femur points OUT and UP 45 deg (knee 26.75 mm above femur axis)
#define GEO_POSE_TIBIA_FROM_VERT_DEG  11.7f   // tibia points down, 11.7 deg outward from vertical
#define GEO_POSE_KNEE_INTERIOR_DEG    56.7f   // angle between femur and tibia links at the knee
#define GEO_POSE_FOOT_OUT_MM          71.07f  // foot pad centre, horizontal from coxa axis
#define GEO_POSE_FOOT_BELOW_FEMUR_MM  46.05f  // foot pad bottom below femur axis
#define GEO_POSE_BODY_CLEARANCE_MM    44.5f   // main plate above ground

// ---------------------------------------------------------------------------------------------
// Servo facts (from CAD + vendor code) relevant to protecting the knees
// ---------------------------------------------------------------------------------------------
// Servo IDENTIFIED 2026-08-18: Hiwonder HPD-0127 mini dual-axis digital servo ($5.99).
// Spec @7.4V: max torque >=1.8 kgf*cm (0.176 N*m) with CLUTCH DISENGAGEMENT at the limit
// (the April "servo grinding" = the clutch slipping), no-load speed <=0.05 s/60deg (1200 deg/s),
// 11 g, plastic gears 1:195, stall current <=1 A, repeatability <=2 deg.
// 280 deg travel over 500..2500 us (SERVO_ANGLE_280_FACTOR = 2000/280 = 7.1429 us/deg)
// Body 13 x 14 x 32.5 mm, shaft on the 14 mm axis, offset 4.7 mm from body centre along the 32.5 mm axis;
// 20 mm round horn (4.5 mm) + 20 mm bearing disc (2.5 mm) on the far side. NOT MG90S-compatible.
#define GEO_SERVO_US_PER_DEG   7.142857f
#define GEO_SERVO_MIN_US       500
#define GEO_SERVO_MAX_US       2500
// Vendor RobotLeg::move() operating points (hiwonder_robot.cpp:153-157):
//   coxa  duty = 140 deg + a  -> 1500 us at a = 0
//   femur duty = 140 deg - b  -> 1500 us at b = 0, then HARD-CLAMPED to 857..2143 (= +-90 deg)
//   tibia duty = 230 deg + c  -> 2143 us at c = 0 (mirrored legs 857): the knee's ZERO is only 50 deg
//                                from the 2500 us mechanical end and there is NO extra clamp on it.
//   => any IK solution asking for c > +50 deg drives the knee servo into its end stop and holds it there.
#define GEO_KNEE_ZERO_DEG      230.0f
#define GEO_KNEE_HEADROOM_DEG   50.0f
