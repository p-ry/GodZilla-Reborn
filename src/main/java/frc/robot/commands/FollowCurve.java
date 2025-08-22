// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.awt.geom.Point2D;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.ArmAssembly;

/**
 * Follow a cubic Bézier path in XY using POSITION-ONLY control.
 *
 * Features:
 *  - Elbow pivot at end of shoulder; forearm mounted via an L-bracket offset (H) that is
 *    PERPENDICULAR TO THE FOREARM AXIS and is forced "ABOVE" (its world Y >= 0).
 *  - Elbow-UP branch only
 *  - Prismatic extension L3 in [0, 350] mm (effective forearm length = L2 + L3)
 *  - Angle convention: world_abs = user - 25°, so user = world_abs + 25°
 *  - Limits: shoulder_user in [0°,115°], 0° < elbow_internal < 180°, shoulder points right/up
 *  - Starts from the live hardware pose: finds closest t on the Bézier
 *  - Smooth ramp from the live pose to IK setpoints (blend time configurable)
 *
 * Units: mm & degrees.
 */
public class FollowCurve extends Command {
  private final ArmAssembly arm;
  private final Point2D.Double p0, p1, p2, p3;
  private final Point2D.Double base;

  // Live feedback hooks (matching existing call-site)
  private final DoubleSupplier shoulderDegNow;
  private final DoubleSupplier elbowDegNow;
  private final DoubleSupplier sliderPosNow;

  private final boolean debug;

  // --- Robot geometry (mm) ---
  private static final double L1 = 527.50;     // shoulder link
  private static final double L2 = 769.0;      // forearm nominal
  private static final double L3_MIN = 0.0;    // prismatic min
  private static final double L3_MAX = 350.0;  // prismatic max
  private static final double H  = 41.917;     // L-bracket offset magnitude (perp to forearm), kept ABOVE

  // Angle mapping: world_abs = user - 25°  (user = world_abs + 25°)
  private static final double USER_OFFSET_DEG = 25.0;

  // Limits
  private static final double SHOULDER_USER_MIN = 2.0;
  private static final double SHOULDER_USER_MAX = 115.0;
  private static final double ELBOW_INT_MIN = 1e-6;       // strictly > 0
  private static final double ELBOW_INT_MAX = 180.0-1e-6; // strictly < 180

  // Motion timing
  private static final double TOTAL_TIME = 5.0; // s
  private static final double DT = 0.02;        // s (plot/update throttle)
  private static final double BLEND_TIME = 0.60; // s ramp-in from live pose

  private final Timer timer = new Timer();
  private double lastT = 0.0;

  // Start location on the curve & start joint snapshot (for smoothing)
  private double startT = 0.0;
  private double startShoulderUser = 0.0;
  private double startElbowInternal = 180.0;
  private double startL3 = 0.0;
  double cmdShoulderUser,rawcmdShoulderUser;
  
  double cmdElbowInt,rawcmdElbowInt;
  // Limit how much the shoulder may change this tick (user degrees)
private static final double MAX_SHOULDER_DELTA_DEG = 5.0;

  
  double cmdL3;

  /**
   * Signature kept to match RobotContainer usage.
   */
  public FollowCurve(
      ArmAssembly arm,
      Point2D.Double startPoint,
      Point2D.Double controlPoint1,
      Point2D.Double controlPoint2,
      Point2D.Double endPoint,
      Point2D.Double base,
      DoubleSupplier shoulderDegNow,
      DoubleSupplier elbowDegNow,
      DoubleSupplier sliderPosNow,
      boolean debug) {

    this.arm = arm;
    this.p0 = startPoint;
    this.p1 = controlPoint1;
    this.p2 = controlPoint2;
    this.p3 = endPoint;
    this.base = base;

    this.shoulderDegNow = shoulderDegNow != null ? shoulderDegNow : () -> 0.0;
    this.elbowDegNow    = elbowDegNow    != null ? elbowDegNow    : () -> 180.0;
    this.sliderPosNow   = sliderPosNow   != null ? sliderPosNow   : () -> 0.0;
    this.debug = debug;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    lastT = 0.0;

    // --- Read current hardware pose (USER deg + mm) ---
    startShoulderUser  = shoulderDegNow.getAsDouble();
    startElbowInternal = elbowDegNow.getAsDouble();
    startL3            = sliderPosNow.getAsDouble();

    // Convert to world-absolute shoulder
    double shoulderAbsDeg = startShoulderUser - USER_OFFSET_DEG;
    double sa = Math.toRadians(shoulderAbsDeg);

    // FK with L-shaped offset kept ABOVE
    double Leff = L2 + clamp(startL3, L3_MIN, L3_MAX);
    double theta2Joint = Math.toRadians(180.0 - startElbowInternal); // from interior

    // Elbow pivot (at end of shoulder link)
    double ex = base.x + L1 * Math.cos(sa);
    double ey = base.y + L1 * Math.sin(sa);

    // Forearm frame
    double th12 = sa + theta2Joint;
    double n2x = -Math.sin(th12), n2y = Math.cos(th12);
    if (n2y < 0.0) { n2x = -n2x; n2y = -n2y; } // enforce "above"
    double u2x = Math.cos(th12), u2y = Math.sin(th12);

    // Wrist (for nearest Bézier point search)
    double wristX = ex + H * n2x + Leff * u2x;
    double wristY = ey + H * n2y + Leff * u2y;

    // --- Find closest point on Bézier to current wrist (coarse, fast) ---
    double bestT = 0.0;
    double bestD2 = Double.MAX_VALUE;
    for (double tt = 0.0; tt <= 1.00001; tt += 0.01) {
      double[] pt = bezier(tt, p0, p1, p2, p3);
      double dx = wristX - pt[0], dy = wristY - pt[1];
      double d2 = dx*dx + dy*dy;
      if (d2 < bestD2) { bestD2 = d2; bestT = tt; }
    }
    startT = clamp(bestT, 0.0, 1.0);

    if (debug) {
      SmartDashboard.putNumber("FollowCurve/start_t", startT);
      SmartDashboard.putNumber("FollowCurve/start_shoulder_user", startShoulderUser);
      SmartDashboard.putNumber("FollowCurve/start_elbow_int", startElbowInternal);
      SmartDashboard.putNumber("FollowCurve/start_L3", startL3);
    }
  }

  @Override
  public void execute() {
    final double t = clamp(startT + (timer.get() / TOTAL_TIME), 0.0, 1.0);
    if (t < lastT + (DT / TOTAL_TIME) && t < 1.0) return; // throttle to ~50 Hz if needed
    lastT = t;

    // Target point in world coords from cubic Bézier
    final double[] xy = bezier(t, p0, p1, p2, p3);
    final double tx = xy[0];
    final double ty = xy[1];

    // Relative to shoulder base
    final double rx = tx - base.x;
    final double ry = ty - base.y;

    // Seed shoulder with current reading (user → world abs)
    double shoulderUserSeed = shoulderDegNow.getAsDouble();
    double shoulderAbsSeed = wrapDeg(shoulderUserSeed - USER_OFFSET_DEG);
    if (!Double.isFinite(shoulderAbsSeed)) {
      shoulderAbsSeed = Math.toDegrees(Math.atan2(ry, rx));
    }

    // Solve IK with limits, elbow-up, L-offset kept above (L3 scan minimal to max)
   // -------- Shoulder-first (±5° user window) --------
double currShoulderUser = clamp(shoulderDegNow.getAsDouble(), SHOULDER_USER_MIN, SHOULDER_USER_MAX);
double minUser = Math.max(SHOULDER_USER_MIN, currShoulderUser - MAX_SHOULDER_DELTA_DEG);
double maxUser = Math.min(SHOULDER_USER_MAX, currShoulderUser + MAX_SHOULDER_DELTA_DEG);

// Search small user range for best elbow/L3 that hits the target
IKResult ik = null;
double bestErr = Double.POSITIVE_INFINITY;

for (double su = minUser; su <= maxUser + 1e-9; su += 1.0) { // 1° steps within ±5°
  double sAbs = su - USER_OFFSET_DEG; // user -> absolute
  IKResult cand = solveWithFixedShoulderAbs(rx, ry, sAbs);
  if (cand == null) continue;

  // Evaluate wrist error (how close cand hits the Cartesian target)
  double th1 = Math.toRadians(cand.shoulderAbsDeg);
  double th2j = Math.toRadians(180.0 - cand.elbowInteriorDeg);

  // elbow pivot at end of L1
  double ex = L1 * Math.cos(th1), ey = L1 * Math.sin(th1);
  // forearm frame
  double th12 = th1 + th2j;
  double n2x = -Math.sin(th12), n2y = Math.cos(th12);
  if (n2y < 0.0) { n2x = -n2x; n2y = -n2y; }
  double u2x = Math.cos(th12), u2y = Math.sin(th12);

  double wx = ex + H*n2x + (L2 + cand.L3mm) * u2x;
  double wy = ey + H*n2y + (L2 + cand.L3mm) * u2y;

  double err = (wx - rx)*(wx - rx) + (wy - ry)*(wy - ry);
  if (err < bestErr) { bestErr = err; ik = cand; }
}

// If nothing feasible inside ±5°, fall back to the original global solver
if (ik == null) {
  ik = solveIkElbowUp_LoffsetAbove(rx, ry, shoulderAbsSeed, /*start*/ false);
  if (ik == null) {
    if (debug) SmartDashboard.putString("FollowCurve/reach", "unreachable/limits");
    return;
  }
}

    if (ik == null) {
        // Fall back to nearest feasible joint set inside constraints
        IKResult nearest = nearestFeasible(rx, ry);
        if (nearest == null) {
          if (debug) SmartDashboard.putString("FollowCurve/reach", "unreachable/no-fallback");
          return;
        }
        ik = nearest;  // use the nearest feasible pose
        if (debug) SmartDashboard.putString("FollowCurve/reach", "fallback-nearest");
      }
      

    // --- Smooth ramp from live pose to IK setpoints ---
    double blend = smooth01(timer.get() / BLEND_TIME);

     cmdShoulderUser = lerpDegShortest(startShoulderUser, ik.shoulderUserDeg, blend);
    cmdElbowInt     = lerpDegShortest(startElbowInternal, ik.elbowInteriorDeg, blend);
     cmdL3           = lerp(startL3, ik.L3mm, blend);

    // Clamp to limits (safety)
    rawcmdShoulderUser = clamp(cmdShoulderUser, SHOULDER_USER_MIN, SHOULDER_USER_MAX);
    rawcmdElbowInt     = clamp(cmdElbowInt,     ELBOW_INT_MIN,     ELBOW_INT_MAX);
    cmdL3           = clamp(cmdL3,           L3_MIN,            L3_MAX);
    

    // Command actuators (POSITION ONLY)
   arm.lowerArm.setDeg( cmdShoulderUser );
arm.upperArm.setDeg( cmdElbowInt );
    //arm.slider.setPos(   cmdL3 );

    if (debug) {
      SmartDashboard.putNumber("FollowCurve/t", t);
      SmartDashboard.putNumber("FollowCurve/tx", tx);
      SmartDashboard.putNumber("FollowCurve/ty", ty);
      SmartDashboard.putNumber("FollowCurve/blend", blend);
      SmartDashboard.putNumber("FollowCurve/cmd_shoulder_user", rawcmdShoulderUser);
      SmartDashboard.putNumber("FollowCurve/cmd_elbow_int", rawcmdElbowInt);
      SmartDashboard.putNumber("FollowCurve/cmd_L3", cmdL3);
    }
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= TOTAL_TIME + 0.05;
  }
@Override
  public void end(boolean interrupted) {
     SmartDashboard.putString("FollowCurve/reach", interrupted ? "interrupted" : "complete");   
    //arm.setJointVelocities(0, 0, 0); // harmless even though we don't use velocities now
   // arm.lowerArm.setPos(cmdShoulderUser);
   // arm.upperArm.setPos(cmdElbowInt); // interior angle
   // arm.slider.setPos(0);
}


  // ----------------- Helpers -----------------

  private static class IKResult {
    final double shoulderAbsDeg;   // absolute (world) from +X
    final double shoulderUserDeg;  // user = world + 25°
    final double elbowInteriorDeg; // 0..180 (internal)
    final double L3mm;             // prismatic extension
    IKResult(double sAbsDeg, double sUserDeg, double eIntDeg, double L3mm) {
      this.shoulderAbsDeg = sAbsDeg;
      this.shoulderUserDeg = sUserDeg;
      this.elbowInteriorDeg = eIntDeg;
      this.L3mm = L3mm;
    }
  }

  /**
   * Elbow-UP IK with L-shaped offset applied perpendicular to the FOREARM axis.
   * We force the offset "ABOVE" by flipping the forearm's left-normal so its world Y >= 0.
   * Limits enforced:
   *   0° ≤ shoulder_user ≤ 115°, 0° < elbow_internal < 180°,
   *   and shoulder must point right/up (cos(theta1_abs) >= 0).
   *
   * If start==true (not used in this version), L3 scan would begin at 0 first tick.
   */
  private static IKResult solveIkElbowUp_LoffsetAbove(double rx, double ry,
                                                      double shoulderAbsSeedDeg,
                                                      boolean start) {
    final double LeffMinScan = L2 + (start ? 0.0 : L3_MIN);
    final double LeffMaxScan = L2 + L3_MAX;

    for (double Leff = LeffMinScan; Leff <= LeffMaxScan; Leff += 1.0) { // 1 mm steps
      double theta1AbsDeg = shoulderAbsSeedDeg;
      boolean okPass = true;
      double theta2 = 0.0;

      // Two-pass de-offset to account for rotated offset H*n2
      for (int pass = 0; pass < 2; pass++) {
        double th1 = Math.toRadians(theta1AbsDeg);

        // vector elbow->target
        double tx = rx - L1 * Math.cos(th1);
        double ty = ry - L1 * Math.sin(th1);
        double r2 = tx*tx + ty*ty;

        // elbow-up (ignore offset this sub-pass)
        double c2 = (r2 - L1*L1 - Leff*Leff) / (2.0 * L1 * Leff);
        if (c2 < -1.0 || c2 > 1.0) { okPass = false; break; }
        theta2 = -Math.acos(clamp(c2, -1.0, 1.0));
        double k1 = L1 + Leff * Math.cos(theta2);
        double k2 = Leff * Math.sin(theta2);
        double theta1 = Math.atan2(ty, tx) - Math.atan2(k2, k1);
        theta1AbsDeg = Math.toDegrees(theta1);

        // Build forearm frame and choose ABOVE normal (n2.y >= 0)
        double th12 = theta1 + theta2;
        double n2x = -Math.sin(th12), n2y = Math.cos(th12);
        if (n2y < 0.0) { n2x = -n2x; n2y = -n2y; }

        // De-offset target and re-solve once
        double adj_tx = tx - H * n2x;
        double adj_ty = ty - H * n2y;
        double adj_r2 = adj_tx*adj_tx + adj_ty*adj_ty;
        c2 = (adj_r2 - L1*L1 - Leff*Leff) / (2.0 * L1 * Leff);
        if (c2 < -1.0 || c2 > 1.0) { okPass = false; break; }
        theta2 = -Math.acos(clamp(c2, -1.0, 1.0));
        k1 = L1 + Leff * Math.cos(theta2);
        k2 = Leff * Math.sin(theta2);
        theta1 = Math.atan2(adj_ty, adj_tx) - Math.atan2(k2, k1);
        theta1AbsDeg = Math.toDegrees(theta1);
      }

      if (!okPass) continue;

      // Angles and limits
      double shoulderAbsDeg = wrapDeg(theta1AbsDeg);
      double shoulderUserDeg = shoulderAbsDeg + USER_OFFSET_DEG;
      double elbowInternalDeg = Math.toDegrees(Math.PI - Math.abs(theta2));

      if (shoulderUserDeg < SHOULDER_USER_MIN - 1e-6 || shoulderUserDeg > SHOULDER_USER_MAX + 1e-6) continue;
      if (!(ELBOW_INT_MIN < elbowInternalDeg && elbowInternalDeg < ELBOW_INT_MAX)) continue;
      if (Math.cos(Math.toRadians(shoulderAbsDeg)) < -1e-9) continue; // must point right or up

      double L3mm = clamp(Leff - L2, L3_MIN, L3_MAX);
      return new IKResult(shoulderAbsDeg, shoulderUserDeg, elbowInternalDeg, L3mm);
    }
    return null;
  }

  /** Shoulder must point right or up. */
private static boolean rightOrUpOk(double shoulderAbsDeg) {
    return Math.cos(Math.toRadians(shoulderAbsDeg)) >= 0.0 - 1e-9;
  }
  
  /** FK for wrist with L-offset ABOVE. Angles are in USER deg for shoulder, INTERNAL deg for elbow. */
  private static double[] fkWrist(double baseX, double baseY, double shoulderUserDeg, double elbowInternalDeg, double L3mm) {
    double th1_abs = Math.toRadians(shoulderUserDeg - USER_OFFSET_DEG); // world_abs
    double th2_joint = Math.toRadians(180.0 - elbowInternalDeg);
  
    double ex = baseX + L1 * Math.cos(th1_abs);
    double ey = baseY + L1 * Math.sin(th1_abs);
  
    double th12 = th1_abs + th2_joint;
    double n2x = -Math.sin(th12), n2y = Math.cos(th12);
    if (n2y < 0.0) { n2x = -n2x; n2y = -n2y; } // keep ABOVE
    double u2x = Math.cos(th12), u2y = Math.sin(th12);
  
    double Leff = L2 + clamp(L3mm, L3_MIN, L3_MAX);
    double wx = ex + H * n2x + Leff * u2x;
    double wy = ey + H * n2y + Leff * u2y;
    return new double[] { wx, wy, ex, ey, ex + H*n2x, ey + H*n2y }; // wrist, elbow pivot, offset point
  }
  
  /** Grid-search a nearest feasible (S_user, E_internal, L3) that minimizes wrist->target distance. */
  private static IKResult nearestFeasible(double rx, double ry) {
    // target in world (relative to base)
    double targetX = rx, targetY = ry;
  
    double bestErr = Double.POSITIVE_INFINITY;
    double bestS = Double.NaN, bestE = Double.NaN, bestL3 = Double.NaN;
  
    // coarse grids (fast)
    for (double su = SHOULDER_USER_MIN; su <= SHOULDER_USER_MAX + 1e-9; su += 5.0) {
      double sAbs = su - USER_OFFSET_DEG;
      if (!rightOrUpOk(sAbs)) continue;
      for (double ei = 1.0; ei < 179.0; ei += 5.0) {
        for (double l3 = L3_MIN; l3 <= L3_MAX + 1e-9; l3 += 25.0) {
          double[] fk = fkWrist(0.0, 0.0, su, ei, l3);  // base at origin since rx,ry already relative
          double dx = fk[0] - targetX, dy = fk[1] - targetY;
          double err = dx*dx + dy*dy;
          if (err < bestErr) { bestErr = err; bestS = su; bestE = ei; bestL3 = l3; }
        }
      }
    }
    if (!Double.isFinite(bestErr)) return null;
  
    // optional fine pass around best (1° / 5 mm)
    for (double su = Math.max(SHOULDER_USER_MIN, bestS-4); su <= Math.min(SHOULDER_USER_MAX, bestS+4); su += 1.0) {
      double sAbs = su - USER_OFFSET_DEG;
      if (!rightOrUpOk(sAbs)) continue;
      for (double ei = Math.max(1.0, bestE-4); ei <= Math.min(179.0, bestE+4); ei += 1.0) {
        for (double l3 = Math.max(L3_MIN, bestL3-15); l3 <= Math.min(L3_MAX, bestL3+15); l3 += 5.0) {
          double[] fk = fkWrist(0.0, 0.0, su, ei, l3);
          double dx = fk[0] - targetX, dy = fk[1] - targetY;
          double err = dx*dx + dy*dy;
          if (err < bestErr) { bestErr = err; bestS = su; bestE = ei; bestL3 = l3; }
        }
      }
    }
  
    double sAbsDeg = bestS - USER_OFFSET_DEG;
    return new IKResult(sAbsDeg, bestS, bestE, bestL3);
  }
  /**
 * Solve with a FIXED shoulder absolute angle (degrees).
 * Returns IKResult or null if no feasible elbow/L3 under constraints.
 */
private static IKResult solveWithFixedShoulderAbs(double rx, double ry, double shoulderAbsDeg) {
  // Try minimal L3 first, then extend up to max
  for (double L3 = L3_MIN; L3 <= L3_MAX; L3 += 1.0) {
    double Leff = L2 + L3;

    // Two-pass de-offset using the fixed shoulder
    double th1 = Math.toRadians(shoulderAbsDeg);

    // First pass: ignore offset to get an elbow guess
    double ex = rx - L1 * Math.cos(th1);
    double ey = ry - L1 * Math.sin(th1);
    double r2 = ex*ex + ey*ey;
    double c2 = (r2 - L1*L1 - Leff*Leff) / (2.0 * L1 * Leff);
    if (c2 < -1.0 || c2 > 1.0) continue;
    double theta2 = -Math.acos(clamp(c2, -1.0, 1.0)); // elbow-up
    double k1 = L1 + Leff * Math.cos(theta2);
    double k2 = Leff * Math.sin(theta2);
    double theta1 = Math.atan2(ey, ex) - Math.atan2(k2, k1); // will deviate from th1 slightly

    // Build forearm frame and choose ABOVE normal
    double th12 = theta1 + theta2;
    double n2x = -Math.sin(th12), n2y = Math.cos(th12);
    if (n2y < 0.0) { n2x = -n2x; n2y = -n2y; }

    // Second pass: de-offset and re-solve with the SAME fixed shoulder angle
    double px = rx - H * n2x - L1 * Math.cos(th1);
    double py = ry - H * n2y - L1 * Math.sin(th1);
    double rr2 = px*px + py*py;
    c2 = (rr2 - L1*L1 - Leff*Leff) / (2.0 * L1 * Leff);
    if (c2 < -1.0 || c2 > 1.0) continue;
    theta2 = -Math.acos(clamp(c2, -1.0, 1.0));
    k1 = L1 + Leff * Math.cos(theta2);
    k2 = Leff * Math.sin(theta2);
    // Now compute theta1 consistent with the fixed shoulder
    theta1 = th1; // force to the requested shoulder

    // Interior elbow angle and user shoulder
    double elbowInterior = Math.toDegrees(Math.PI - Math.abs(theta2));
    double shoulderUserDeg = shoulderAbsDeg + USER_OFFSET_DEG;

    // Limits
    if (shoulderUserDeg < SHOULDER_USER_MIN - 1e-6 || shoulderUserDeg > SHOULDER_USER_MAX + 1e-6) continue;
    if (!(ELBOW_INT_MIN < elbowInterior && elbowInterior < ELBOW_INT_MAX)) continue;
    if (Math.cos(th1) < -1e-9) continue; // must point right or straight up

    return new IKResult(shoulderAbsDeg, shoulderUserDeg, elbowInterior, L3);
  }
  return null;
}



  private static double[] bezier(double t, Point2D.Double a, Point2D.Double b, Point2D.Double c, Point2D.Double d) {
    double u = 1.0 - t;
    double uu = u*u;
    double tt = t*t;
    double uuu = uu*u;
    double ttt = tt*t;

    double x = uuu*a.x + 3*uu*t*b.x + 3*u*tt*c.x + ttt*d.x;
    double y = uuu*a.y + 3*uu*t*b.y + 3*u*tt*c.y + ttt*d.y;
    return new double[]{x, y};
  }

  private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

  private static double wrapDeg(double deg) {
    double d = deg % 360.0;
    if (d < -180) d += 360.0;
    if (d >= 180) d -= 360.0;
    return d;
  }

  private static double lerp(double a, double b, double t) { return a + t * (b - a); }

  /** Shortest-path angle lerp in degrees (wrap-safe). */
  private static double lerpDegShortest(double aDeg, double bDeg, double t) {
    double d = wrapDeg(bDeg - aDeg);
    return aDeg + t * d;
  }

  /** Smoothstep easing 0..1 -> 0..1. */
  private static double smooth01(double x) {
    double t = clamp(x, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
  }
}
