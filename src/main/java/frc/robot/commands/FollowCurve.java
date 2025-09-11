// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.awt.geom.Point2D;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Ace;
import frc.robot.subsystems.ArmAssembly;
import frc.robot.Constants;

/**
 * Follow a cubic Bézier path in XY using POSITION-ONLY control.
 *
 * Features:
 * - Elbow pivot at end of shoulder; forearm mounted via an L-bracket offset (H)
 * that is
 * PERPENDICULAR TO THE FOREARM AXIS and is forced "ABOVE" (its world Y >= 0).
 * - Elbow-UP branch only (but we now test both ±acos branches and choose the
 * better)
 * - Prismatic extension L3 in [0, 350] mm (effective forearm length = L2 + L3)
 * - Angle convention: world_abs = user - 25°, so user = world_abs + 25°
 * - Limits: shoulder_user in [0°,115°], 0° < elbow_internal < 172° (hard cap),
 * shoulder points right/up
 * - Starts from the live hardware pose: finds closest t on the Bézier
 * - Smooth ramp from the live pose to IK setpoints (blend time configurable)
 * - Soft penalties to avoid near-straight elbow and to prefer a more upright
 * shoulder near the end
 *
 * Units: mm & degrees.
 */
public class FollowCurve extends Command {
    private final ArmAssembly arm;
    private final Ace ace; // used in end()
    private final Point2D.Double p0, p1, p2, p3;
    private final Point2D.Double base;

    // Live feedback hooks (matching existing call-site)
    private final DoubleSupplier shoulderDegNow;
    private final DoubleSupplier elbowDegNow;
    private final DoubleSupplier sliderPosNow;

    // Scales how fast we march along the Bézier (1.0 = original speed)
    private static final double PATH_SPEED = 1.0; // try 1.5–2.0 for faster motion

    private final boolean debug;

    // --- Robot geometry (mm) ---
    private static final double L1 = 527.50; // shoulder link
    private static final double L2 = 769.0; // forearm nominal
    private static final double L3_MIN = 0.0; // prismatic min
    private static final double L3_MAX = 350.0; // prismatic max
    private static final double H = 41.917; // L-bracket offset magnitude (perp to forearm), kept ABOVE

    /// One source of truth:
    private static final double USER_OFFSET_DEG = 25.0; // user = abs + 25

    private static double toAbs(double shoulderUserDeg) {
        return shoulderUserDeg - USER_OFFSET_DEG;
    }

    private static double toUser(double shoulderAbsDeg) {
        return shoulderAbsDeg + USER_OFFSET_DEG;
    }

    // Limits
    private static final double SHOULDER_USER_MIN = 0.0;
    private static final double SHOULDER_USER_MAX = 105.0;
    private static final double ELBOW_INT_MIN = 1e-6; // strictly > 0
    // HARD elbow ceiling (never exceed)
    private static final double ELBOW_INT_HARD_MAX = 172.0; // deg
    // Soft elbow preference (prefer not to go past this)
    private static final double ELBOW_INT_SOFT_MAX = 158.0; // deg (was 168.0) // deg
    // Tie existing max to hard cap so all checks use the ceiling
    private static final double ELBOW_INT_MAX = ELBOW_INT_HARD_MAX;

    // Soft penalty weights / preferences
    private static final double ELBOW_PENALTY_W = 2.5e4; // mm^2 per (deg over soft)^2
    private static final double SHOULDER_USER_SOFT_MIN = 95.0; // (legacy; not used in bias now)
    private static final double SHOULDER_USER_SOFT_MAX = 90.0; // prefer ≤ ~90° near end (10° less) // prefer ≥ ~95°
                                                               // user near end
    private static final double SHOULDER_SOFT_PENALTY_W = 7.5e3; // mm^2 per (deg under)^2
    private static final double L3_SOFT_MAX = 0.85 * L3_MAX; // prefer at least ~65% extension near end // prefer ≤ 60%
                                                             // of max extension
    private static final double L3_SOFT_PENALTY_W = 30.0; // mm^2 per (mm over)^2

    // Motion timing
    private static final double TOTAL_TIME = 1.0; // s
    private static final double DT = 0.02; // s (plot/update throttle)
    private static final double BLEND_TIME = 0.20; // s ramp-in from live pose

    private final Timer timer = new Timer();
    private double lastT = 0.0;

    // Start location on the curve & start joint snapshot (for smoothing)
    private double startT = 0.0;
    private double startShoulderUser = 0.0;
    private double startElbowInternal = 180.0;
    private double startL3 = 0.0;
    static double shoulderBiasPhase = 0.0;
    static double targetElbowInt = 10.0; // target elbow interior angle (for biasing)
    double cmdShoulderUser, rawcmdShoulderUser;
    double cmdElbowInt, rawcmdElbowInt;
    double cmdL3;

    // If best wrist error (to the Bezier target) exceeds this, widen the shoulder
    // window (mm^2)
    private static final double IK_ERR_THRESH_MM2 = 50.0 * 50.0; // ~50 mm radial error

    // Limit how much the shoulder may change this tick (user degrees) — currently
    // unused but kept
    private static final double MAX_SHOULDER_DELTA_DEG = 9.0;

    /**
     * Signature kept to match RobotContainer usage.
     */
    public FollowCurve(
            ArmAssembly arm,
            Ace ace,
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
        this.ace = ace;
        this.p0 = startPoint;
        this.p1 = controlPoint1;
        this.p2 = controlPoint2;
        this.p3 = endPoint;
        this.base = base;

        this.shoulderDegNow = shoulderDegNow != null ? shoulderDegNow : () -> 0.0;
        this.elbowDegNow = elbowDegNow != null ? elbowDegNow : () -> 180.0;
        this.sliderPosNow = sliderPosNow != null ? sliderPosNow : () -> 0.0;
        this.debug = debug;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        Constants.followCurve = true; // enable FollowCurve mode
        timer.reset();
        timer.start();
        lastT = 0.0;

        // --- Read current hardware pose (USER deg + mm) ---
        startShoulderUser = shoulderDegNow.getAsDouble();
        startElbowInternal = elbowDegNow.getAsDouble();
        startL3 = sliderPosNow.getAsDouble();

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
        if (n2y < 0.0) {
            n2x = -n2x;
            n2y = -n2y;
        } // enforce "above"
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
            double d2 = dx * dx + dy * dy;
            if (d2 < bestD2) {
                bestD2 = d2;
                bestT = tt;
            }
        }
        startT = clamp(bestT, 0.0, 1.0);

        if (Constants.debug) {
            SmartDashboard.putNumber("FollowCurve/start_t", startT);
            SmartDashboard.putNumber("FollowCurve/start_shoulder_user", startShoulderUser);
            SmartDashboard.putNumber("FollowCurve/start_elbow_int", startElbowInternal);
            SmartDashboard.putNumber("FollowCurve/start_L3", startL3);
        }
    }

    @Override
    public void execute() {
        final double t = clamp(startT + (PATH_SPEED * (timer.get() / TOTAL_TIME)), 0.0, 1.0);

        // throttle to ~50 Hz
        if (t < lastT + (DT / TOTAL_TIME) && t < 1.0)
            return;
        lastT = t;
        // Bias only near the end so we don't pull the shoulder down at t≈0
        shoulderBiasPhase = smooth01((t - 0.50) / 0.50); // 0→1 as t goes 0.5→1.0

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

        // -------- Shoulder-first with ADAPTIVE window --------
        double currShoulderUser = clamp(shoulderDegNow.getAsDouble(), SHOULDER_USER_MIN, SHOULDER_USER_MAX);

        // Candidate half-spans to try (deg): ±5, ±15, ±30, then full range
        final double[] halfSpans = new double[] { 5.0, 15.0, 30.0, 999.0 };

        IKResult ik = null;
        double bestErr = Double.POSITIVE_INFINITY;

        for (double span : halfSpans) {
            double minUser = Math.max(SHOULDER_USER_MIN, currShoulderUser - span);
            double maxUser = Math.min(SHOULDER_USER_MAX, currShoulderUser + span);

            IKResult localBest = null;
            double localBestErr = Double.POSITIVE_INFINITY;

            for (double su = minUser; su <= maxUser + 1e-9; su += 1.0) {
                double sAbs = su - USER_OFFSET_DEG; // user -> absolute
                IKResult cand = solveWithFixedShoulderAbsBestBranch(rx, ry, sAbs);
                if (cand == null)
                    continue;

                // Evaluate wrist error to the RELATIVE Bezier target (rx, ry)
                double th1 = Math.toRadians(cand.shoulderAbsDeg);
                double th2j = Math.toRadians(180.0 - cand.elbowInteriorDeg);

                // Elbow pivot at end of L1
                double ex_rel = L1 * Math.cos(th1);
                double ey_rel = L1 * Math.sin(th1);

                // Forearm frame
                double th12 = th1 + th2j;
                double n2x = -Math.sin(th12), n2y = Math.cos(th12);
                if (n2y < 0.0) {
                    n2x = -n2x;
                    n2y = -n2y;
                }
                double u2x = Math.cos(th12), u2y = Math.sin(th12);

                double wx_rel = ex_rel + H * n2x + (L2 + cand.L3mm) * u2x;
                double wy_rel = ey_rel + H * n2y + (L2 + cand.L3mm) * u2y;

                double err = (wx_rel - rx) * (wx_rel - rx) + (wy_rel - ry) * (wy_rel - ry);

                // Soft penalty for near-straight elbow (prefer ≤ ELBOW_INT_SOFT_MAX)
                double elbowInt = cand.elbowInteriorDeg;
                if (elbowInt > ELBOW_INT_SOFT_MAX) {
                    double over = elbowInt - ELBOW_INT_SOFT_MAX;
                    err += ELBOW_PENALTY_W * over * over;
                }
                // Prefer slightly less shoulder near end of path (soft max)
                double suUser = cand.shoulderUserDeg;
                if (suUser > SHOULDER_USER_SOFT_MAX) {
                    double over = suUser - SHOULDER_USER_SOFT_MAX;
                    err += shoulderBiasPhase * SHOULDER_SOFT_PENALTY_W * over * over; // gated
                }

                // NOTE: No L3 bias here — we want L3 to actuate LAST during blending.

                if (err < localBestErr) {
                    localBestErr = err;
                    localBest = cand;
                }
            }

            if (localBest != null) {
                ik = localBest;
                bestErr = localBestErr;
                if (bestErr <= IK_ERR_THRESH_MM2)
                    break;
            }
        }

        // Fallback if adaptive search fails
        if (ik == null) {
            ik = solveIkElbowUp_LoffsetAbove(rx, ry, shoulderAbsSeed, /* start */ false);
            if (ik == null) {
                if (debug)
                    SmartDashboard.putString("FollowCurve/reach", "unreachable/limits");
                return;
            }
        }

        // --- Smooth ramp from live pose to IK setpoints ---
        // Angles ramp immediately (as before)
        final double blendAng = smooth01(timer.get() / BLEND_TIME);
        double targetShoulderUser = ik.shoulderUserDeg;

        // For the first ~0.30 s, don't command shoulder below the startup reading
        if (timer.get() < 0.30 && targetShoulderUser < startShoulderUser) {
            targetShoulderUser = startShoulderUser;
        }

        cmdShoulderUser = lerpDegShortest(startShoulderUser, targetShoulderUser, blendAng);
       if(Constants.debug){
         SmartDashboard.putBoolean("FollowCurve/noDip_active", timer.get() < 0.30);
       }
        cmdElbowInt = lerpDegShortest(startElbowInternal, ik.elbowInteriorDeg, blendAng);

        // L3 moves LAST: hold until late in the path, then ramp to target
        final double L3_START_T = 0.65; // start extending slider near the end
        final double L3_END_T = 1.00; // finish at path end
        double l3Phase = clamp((t - L3_START_T) / (L3_END_T - L3_START_T), 0.0, 1.0);
        double blendL3 = smooth01(l3Phase);
        cmdL3 = lerp(startL3, ik.L3mm, blendL3);

        // Clamp to limits (safety)
        rawcmdShoulderUser = clamp(cmdShoulderUser, SHOULDER_USER_MIN, SHOULDER_USER_MAX);
        rawcmdElbowInt = clamp(cmdElbowInt, ELBOW_INT_MIN, ELBOW_INT_MAX);
        cmdL3 = clamp(cmdL3, L3_MIN, L3_MAX);

        // --- existing clamp & commands above this point ---

        // --- EASY ELBOW NO-DIP GUARD (first 0.30 s) ---
        final double NO_DIP_WINDOW_S = 0.30;
        if (timer.get() < NO_DIP_WINDOW_S && rawcmdElbowInt < startElbowInternal) {
            rawcmdElbowInt = startElbowInternal;
        }

        // Command actuators (POSITION ONLY)
        arm.lowerArm.setDeg(rawcmdShoulderUser);
        arm.upperArm.setDeg(rawcmdElbowInt);
        arm.slider.setMM(cmdL3);

        // ===== Real-world vs Commanded FK =====
        // Read back *measured* joints (USER deg for shoulder, INTERNAL deg for elbow,
        // mm for L3)
        double measShoulderUser = shoulderDegNow.getAsDouble();
        double measElbowInt = elbowDegNow.getAsDouble();
        double measL3mm = sliderPosNow.getAsDouble();

        // FK using *measured* joints → this is your real-world wrist pose
        double[] fkWorldMeas = fkWrist(base.x, base.y, measShoulderUser, measElbowInt, measL3mm);

        // FK using *commanded* joints → target/intent pose (what you asked for)
        double[] fkWorldCmd = fkWrist(base.x, base.y, rawcmdShoulderUser, rawcmdElbowInt, cmdL3);

        // Optional: error to current Bézier target (rx,ry are target minus base)
        double dx_meas = fkWorldMeas[0] - (base.x + (tx - base.x));
        double dy_meas = fkWorldMeas[1] - (base.y + (ty - base.y));
        double err_meas_mm2 = dx_meas * dx_meas + dy_meas * dy_meas;

        double dx_cmd = fkWorldCmd[0] - (base.x + (tx - base.x));
        double dy_cmd = fkWorldCmd[1] - (base.y + (ty - base.y));
        double err_cmd_mm2 = dx_cmd * dx_cmd + dy_cmd * dy_cmd;

        // ---- Dashboard keys (clear & explicit) ----

        if (Constants.debug) {
            SmartDashboard.putNumber("FollowCurve/wrist_x_meas", fkWorldMeas[0]);
            SmartDashboard.putNumber("FollowCurve/wrist_y_meas", fkWorldMeas[1]);
            SmartDashboard.putNumber("FollowCurve/wrist_x_cmd", fkWorldCmd[0]);
            SmartDashboard.putNumber("FollowCurve/wrist_y_cmd", fkWorldCmd[1]);
            SmartDashboard.putNumber("FollowCurve/err_meas_mm2", err_meas_mm2);
            SmartDashboard.putNumber("FollowCurve/err_cmd_mm2", err_cmd_mm2);
            SmartDashboard.putNumber("FollowCurve/t", t);
            SmartDashboard.putNumber("FollowCurve/tx", tx);
            SmartDashboard.putNumber("FollowCurve/ty", ty);
            SmartDashboard.putNumber("FollowCurve/blendAng", blendAng);
            SmartDashboard.putNumber("FollowCurve/l3_blend", blendL3);
            SmartDashboard.putNumber("FollowCurve/cmd_shoulder_user", rawcmdShoulderUser);
            SmartDashboard.putNumber("FollowCurve/cmd_elbow_int", rawcmdElbowInt);
            SmartDashboard.putNumber("FollowCurve/cmd_L3", cmdL3);
            SmartDashboard.putNumber("FollowCurve/ik_err_mm2", bestErr);
            SmartDashboard.putNumber("FollowCurve/ik_elbow_int_deg", ik.elbowInteriorDeg);
            SmartDashboard.putNumber("FollowCurve/ik_shoulder_user_deg", ik.shoulderUserDeg);
            SmartDashboard.putNumber("FollowCurve/ik_L3_mm", ik.L3mm);
        SmartDashboard.putBoolean("debug", debug);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= TOTAL_TIME + 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("FollowCurve/reach", interrupted ? "interrupted" : "complete");
        ace.setPos(3.0);
        arm.wrist.setPos(9.4); // example wrist reset
        Constants.followCurve = false; // disable FollowCurve mode
    }

    // ----------------- Helpers -----------------

    private static class IKResult {
        final double shoulderAbsDeg; // absolute (world) from +X
        final double shoulderUserDeg; // user = world + 25°
        final double elbowInteriorDeg; // 0..180 (internal)
        final double L3mm; // prismatic extension

        IKResult(double sAbsDeg, double sUserDeg, double eIntDeg, double L3mm) {
            this.shoulderAbsDeg = sAbsDeg;
            this.shoulderUserDeg = sUserDeg;
            this.elbowInteriorDeg = eIntDeg;
            this.L3mm = L3mm;
        }
    }

    /**
     * Elbow-UP IK with L-shaped offset applied perpendicular to the FOREARM axis.
     * We force the offset "ABOVE" by flipping the forearm's left-normal so its
     * world Y >= 0.
     * Limits enforced:
     * 0° ≤ shoulder_user ≤ 115°, 0° < elbow_internal < 172° (hard),
     * and shoulder must point right/up (cos(theta1_abs) >= 0).
     */
    private static IKResult solveIkElbowUp_LoffsetAbove(double rx, double ry, double shoulderAbsSeedDeg,
            boolean start) {
        final double LeffMinScan = L2 + (start ? 0.0 : L3_MIN);
        final double LeffMaxScan = L2 + L3_MAX;

        for (double Leff = LeffMinScan; Leff <= LeffMaxScan; Leff += 1.0) {
            double theta1AbsDeg = shoulderAbsSeedDeg;
            boolean okPass = true;
            double theta2 = 0.0;

            for (int pass = 0; pass < 2; pass++) {
                double th1 = Math.toRadians(theta1AbsDeg);

                // shoulder → target (corrected)
                double tx = rx, ty = ry;
                double r2 = tx * tx + ty * ty;

                double c2 = (r2 - L1 * L1 - Leff * Leff) / (2.0 * L1 * Leff);
                if (c2 < -1.0 || c2 > 1.0) {
                    okPass = false;
                    break;
                }
                theta2 = -Math.acos(clamp(c2, -1.0, 1.0));
                double k1 = L1 + Leff * Math.cos(theta2);
                double k2 = Leff * Math.sin(theta2);
                double theta1 = Math.atan2(ty, tx) - Math.atan2(k2, k1);
                theta1AbsDeg = Math.toDegrees(theta1);

                double th12 = theta1 + theta2;
                double n2x = -Math.sin(th12), n2y = Math.cos(th12);
                if (n2y < 0.0) {
                    n2x = -n2x;
                    n2y = -n2y;
                }

                double adj_tx = tx - H * n2x;
                double adj_ty = ty - H * n2y;
                double adj_r2 = adj_tx * adj_tx + adj_ty * adj_ty;
                c2 = (adj_r2 - L1 * L1 - Leff * Leff) / (2.0 * L1 * Leff);
                if (c2 < -1.0 || c2 > 1.0) {
                    okPass = false;
                    break;
                }
                theta2 = -Math.acos(clamp(c2, -1.0, 1.0));
                k1 = L1 + Leff * Math.cos(theta2);
                k2 = Leff * Math.sin(theta2);
                theta1 = Math.atan2(adj_ty, adj_tx) - Math.atan2(k2, k1);
                theta1AbsDeg = Math.toDegrees(theta1);
            }

            if (!okPass)
                continue;

            double shoulderAbsDeg = wrapDeg(theta1AbsDeg);
            double shoulderUserDeg = shoulderAbsDeg + USER_OFFSET_DEG;
            double elbowInternalDeg = Math.toDegrees(Math.PI - Math.abs(theta2));

            if (shoulderUserDeg < SHOULDER_USER_MIN - 1e-6 || shoulderUserDeg > SHOULDER_USER_MAX + 1e-6)
                continue;
            if (!(ELBOW_INT_MIN < elbowInternalDeg && elbowInternalDeg < ELBOW_INT_HARD_MAX))
                continue;
            if (Math.cos(Math.toRadians(shoulderAbsDeg)) < -1e-9)
                continue;

            double L3mm = clamp(Leff - L2, L3_MIN, L3_MAX);
            return new IKResult(shoulderAbsDeg, shoulderUserDeg, elbowInternalDeg, L3mm);
        }
        return null;
    }

    /** Shoulder must point right or up. */
    private static boolean rightOrUpOk(double shoulderAbsDeg) {
        return Math.cos(Math.toRadians(shoulderAbsDeg)) >= 0.0 - 1e-9;
    }

    /**
     * FK for wrist with L-offset ABOVE. Angles are in USER deg for shoulder,
     * INTERNAL deg for elbow.
     */
    private static double[] fkWrist(double baseX, double baseY, double shoulderUserDeg, double elbowInternalDeg,
            double L3mm) {
        double th1_abs = Math.toRadians(shoulderUserDeg - USER_OFFSET_DEG); // world_abs
        double th2_joint = Math.toRadians(180.0 - elbowInternalDeg);

        double ex = baseX + L1 * Math.cos(th1_abs);
        double ey = baseY + L1 * Math.sin(th1_abs);

        double th12 = th1_abs + th2_joint;
        double n2x = -Math.sin(th12), n2y = Math.cos(th12);
        if (n2y < 0.0) {
            n2x = -n2x;
            n2y = -n2y;
        } // keep ABOVE
        double u2x = Math.cos(th12), u2y = Math.sin(th12);

        double Leff = L2 + clamp(L3mm, L3_MIN, L3_MAX);
        double wx = ex + H * n2x + Leff * u2x;
        double wy = ey + H * n2y + Leff * u2y;
        return new double[] { wx, wy, ex, ey, ex + H * n2x, ey + H * n2y }; // wrist, elbow pivot, offset point
    }

    /**
     * Grid-search a nearest feasible (S_user, E_internal, L3) that minimizes
     * wrist->target distance, with soft penalties.
     */
    private static IKResult nearestFeasible(double rx, double ry) {
        // target in world (relative to base)
        double targetX = rx, targetY = ry;

        double bestErr = Double.POSITIVE_INFINITY;
        double bestS = Double.NaN, bestE = Double.NaN, bestL3 = Double.NaN;

        // coarse grids (fast)
        for (double su = SHOULDER_USER_MIN; su <= SHOULDER_USER_MAX + 1e-9; su += 5.0) {
            double sAbs = su - USER_OFFSET_DEG;
            if (!rightOrUpOk(sAbs))
                continue;
            for (double ei = Math.max(1.0, ELBOW_INT_MIN + 1.0); ei < Math.min(179.0, ELBOW_INT_HARD_MAX); ei += 5.0) {
                for (double l3 = L3_MIN; l3 <= L3_MAX + 1e-9; l3 += 25.0) {
                    double[] fk = fkWrist(0.0, 0.0, su, ei, l3); // base at origin since rx,ry already relative
                    double dx = fk[0] - targetX, dy = fk[1] - targetY;
                    double err = dx * dx + dy * dy;

                    // elbow soft penalty
                    if (ei > ELBOW_INT_SOFT_MAX) {
                        double over = ei - ELBOW_INT_SOFT_MAX;
                        err += ELBOW_PENALTY_W * over * over;
                    }
                    // shoulder soft-max (full weight in fallback)
                    if (su > SHOULDER_USER_SOFT_MAX) {
                        double over = su - SHOULDER_USER_SOFT_MAX;
                        err += shoulderBiasPhase * SHOULDER_SOFT_PENALTY_W * over * over;
                    }
                    // L3 soft-min
                    double underL3 = Math.max(0.0, L3_SOFT_MAX - l3);
                    if (underL3 > 0.0) {
                        err += L3_SOFT_PENALTY_W * underL3 * underL3;
                    }

                    if (err < bestErr) {
                        bestErr = err;
                        bestS = su;
                        bestE = ei;
                        bestL3 = l3;
                    }
                }
            }
        }
        if (!Double.isFinite(bestErr))
            return null;

        // optional fine pass around best (1° / 5 mm)
        for (double su = Math.max(SHOULDER_USER_MIN, bestS - 4); su <= Math.min(SHOULDER_USER_MAX,
                bestS + 4); su += 1.0) {
            double sAbs = su - USER_OFFSET_DEG;
            if (!rightOrUpOk(sAbs))
                continue;
            for (double ei = Math.max(1.0, bestE - 4); ei <= Math.min(ELBOW_INT_HARD_MAX - 1e-6,
                    bestE + 4); ei += 1.0) {
                for (double l3 = Math.max(L3_MIN, bestL3 - 15); l3 <= Math.min(L3_MAX, bestL3 + 15); l3 += 5.0) {
                    double[] fk = fkWrist(0.0, 0.0, su, ei, l3);
                    double dx = fk[0] - targetX, dy = fk[1] - targetY;
                    double err = dx * dx + dy * dy;

                    if (ei > ELBOW_INT_SOFT_MAX) {
                        double over = ei - ELBOW_INT_SOFT_MAX;
                        err += ELBOW_PENALTY_W * over * over;
                    }
                    if (su < SHOULDER_USER_SOFT_MIN) {
                        double under = SHOULDER_USER_SOFT_MIN - su;
                        err += shoulderBiasPhase * SHOULDER_SOFT_PENALTY_W * under * under;
                    }
                    double overL3 = Math.max(0.0, l3 - L3_SOFT_MAX);
                    if (overL3 > 0.0) {
                        err += L3_SOFT_PENALTY_W * overL3 * overL3;
                    }

                    if (err < bestErr) {
                        bestErr = err;
                        bestS = su;
                        bestE = ei;
                        bestL3 = l3;
                    }
                }
            }
        }

        double sAbsDeg = bestS - USER_OFFSET_DEG;
        return new IKResult(sAbsDeg, bestS, bestE, bestL3);
    }

    /** Weighted error used by the best-branch chooser */
    private static double candidateErrMm2(double rx, double ry, IKResult ik) {
        if (ik == null)
            return Double.POSITIVE_INFINITY;
        double[] fk = fkWrist(0.0, 0.0, ik.shoulderUserDeg, ik.elbowInteriorDeg, ik.L3mm);
        double dx = fk[0] - rx, dy = fk[1] - ry;
        double err = dx * dx + dy * dy;
        if (ik.elbowInteriorDeg > ELBOW_INT_SOFT_MAX) {
            double over = ik.elbowInteriorDeg - ELBOW_INT_SOFT_MAX;
            err += ELBOW_PENALTY_W * over * over;
        }
        return err;
    }

    /**
     * Try both elbow branches (±acos) with fixed shoulder; return the better one.
     */
    private static IKResult solveWithFixedShoulderAbsBestBranch(double rx, double ry, double shoulderAbsDeg) {
        IKResult a = solveWithFixedShoulderAbs(rx, ry, shoulderAbsDeg, -1); // one branch
        IKResult b = solveWithFixedShoulderAbs(rx, ry, shoulderAbsDeg, +1); // opposite branch
        double ea = candidateErrMm2(rx, ry, a);
        double eb = candidateErrMm2(rx, ry, b);
        return (ea <= eb) ? a : b;
    }

    /** Fixed-shoulder solver that seeds the elbow with the chosen sign (±acos). */
    private static IKResult solveWithFixedShoulderAbs(double rx, double ry, double shoulderAbsDeg, int elbowSign) {
        final double th1 = Math.toRadians(shoulderAbsDeg);
        for (double L3 = L3_MAX; L3 >= L3_MIN - 1e-9; L3 -= 1.0) {
            double Leff = L2 + L3;

            // Pass 1: get a usable normal
            double r2 = rx * rx + ry * ry;
            double c2 = (r2 - L1 * L1 - Leff * Leff) / (2.0 * L1 * Leff);
            if (c2 < -1.0 || c2 > 1.0)
                continue;

            double theta2 = elbowSign * Math.acos(clamp(c2, -1.0, 1.0));
            double th12 = th1 + theta2;

            double n2x = -Math.sin(th12), n2y = Math.cos(th12);
            if (n2y < 0.0) {
                n2x = -n2x;
                n2y = -n2y;
            } // keep offset ABOVE

            // Pass 2: de-offset by H then re-solve with same branch
            double px = rx - H * n2x, py = ry - H * n2y;
            double rr2 = px * px + py * py;
            double c2b = (rr2 - L1 * L1 - Leff * Leff) / (2.0 * L1 * Leff);
            if (c2b < -1.0 || c2b > 1.0)
                continue;

            theta2 = elbowSign * Math.acos(clamp(c2b, -1.0, 1.0));

            double elbowInterior = Math.toDegrees(Math.PI - Math.abs(theta2));
            double shoulderUserDeg = shoulderAbsDeg + USER_OFFSET_DEG;

            if (shoulderUserDeg < SHOULDER_USER_MIN - 1e-6 || shoulderUserDeg > SHOULDER_USER_MAX + 1e-6)
                continue;
            if (!(ELBOW_INT_MIN < elbowInterior && elbowInterior < ELBOW_INT_HARD_MAX))
                continue;
            if (!rightOrUpOk(shoulderAbsDeg))
                continue;

            return new IKResult(shoulderAbsDeg, shoulderUserDeg, elbowInterior, L3);
        }
        return null;
    }

    private static double[] bezier(double t, Point2D.Double a, Point2D.Double b, Point2D.Double c, Point2D.Double d) {
        double u = 1.0 - t;
        double uu = u * u;
        double tt = t * t;
        double uuu = uu * u;
        double ttt = tt * t;

        double x = uuu * a.x + 3 * uu * t * b.x + 3 * u * tt * c.x + ttt * d.x;
        double y = uuu * a.y + 3 * uu * t * b.y + 3 * u * tt * c.y + ttt * d.y;
        return new double[] { x, y };
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrapDeg(double deg) {
        double d = deg % 360.0;
        if (d < -180)
            d += 360.0;
        if (d >= 180)
            d -= 360.0;
        return d;
    }

    private static double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }

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
