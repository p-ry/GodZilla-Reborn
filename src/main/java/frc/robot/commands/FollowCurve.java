package frc.robot.commands;

import java.util.List;
import java.awt.geom.Point2D;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAssembly;
import frc.robot.BezierLogger;
import frc.robot.BezierPlotTool;
import frc.robot.RobotContainer;
import frc.robot.Utilitys.BezierCurve;
import frc.robot.BezierCurveJava;
import frc.robot.Vector2D;
import java.awt.geom.Point2D;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class FollowCurve extends Command {
    private final ArmAssembly arm;
    private final BezierCurveJava curve;
    private final Point2D.Double base;

    private static final double totalTime           = 5.0;
    private static final double dt                  = 0.02;
    private double time;

    // arm geometry & limits (use consistent length units with your path points)
    private static final double L1                 = 496.2;    // upper arm length
    private static final double L2                 = 696.9;    // forearm base length (no slider)
    private static final double MAX_SHOULDER_DEG   = 70.0;     // mechanical limit (deg)
    private static final double MAX_SHOULDER_VEL   = 40.0;      // deg/s command cap
    private static final double MAX_ELBOW_VEL      = 20.0;      // deg/s command cap
    private static final double MAX_SLIDER_LENGTH  = 350.0;    // slider stroke (same units as L1/L2)
    private static final double MAX_SLIDER_RPS     = 1.0;      // command cap
    private static final double SLIDER_UNITS_PER_REV = 0.142875; // distance units per rev

     // ===== Direction & smoothing =====
     private static final double ELBOW_SIGN = -1.0;           // invert elbow as you observed
     private static final double VEL_DB_SHOULDER = 0.05;      // deg/s deadband
     private static final double VEL_DB_ELBOW    = 0.05;      // deg/s deadband
     private static final double VEL_DB_SLIDER   = 0.005;     // rps deadband
     private static final double ANG_TOL_DEG     = 0.25;      // stop window (angles)
     private static final double SLIDER_TOL      = 0.5;       // stop window (distance units)
     private static final double SP_TOL          = 1e-3;      // smoothstep' threshold to call it "at end"
 
 
     // ===== Final target (for stop window / creep kill) =====
     private Point2D.Double finalPos;
     private double finalRawShoulderDeg;
     private double finalElbowDeg;
     private double finalSliderUnits;
    // state for finite-difference
    private double lastRawShoulderDeg;
    private double lastElbowDeg;
    double rawShoulderDeg;
    
    double elbowDeg;
    
    double sliderUnits;
    

    private double lastSliderUnits;

    public FollowCurve(
            ArmAssembly arm,
            Point2D.Double p0, Point2D.Double p1,
            Point2D.Double p2, Point2D.Double p3,
            Point2D.Double base) {
        this.arm    = arm;
        this.curve  = new BezierCurveJava(p0, p1, p2, p3);
        this.base   = base;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        time = 0;
        Point2D.Double startPos = curve.getPositionAtArcLengthTime(0.0);
        lastRawShoulderDeg  = computeRawShoulderDeg(startPos);
        lastElbowDeg        = computeElbowDeg(startPos);
        lastSliderUnits     = computeSlider(startPos);
    }

    @Override
    public void execute() {
        if (time >= 1.0) { arm.setJointVelocities(0, 0, 0); return; }

        // === 1) Time-warp and sampling ===
        double s  = smoothstep(time);           // position progress (arc-length parameter)
        double sp = smoothstepDeriv(time);      // ds/du (zero at ends)

        Point2D.Double pos = curve.getPositionAtArcLengthTime(s);
        Point2D.Double dPos_ds = curve.getVelocityAtArcLengthTime(s); // ∂pos/∂s

        // For now assume base uDot = 1/totalTime (we'll scale by 'g' later)
        double dxdT_base = dPos_ds.x * (sp / totalTime);
        double dydT_base = dPos_ds.y * (sp / totalTime);

        // Current kinematics at pose (angles depend only on pos, not uDot)
        double rawShoulderDeg = computeRawShoulderDeg(pos);
        double elbowDeg       = computeElbowDeg(pos);
        double sliderUnits    = computeSlider(pos);

        // Predict slider rate for base uDot using finite diff over one unsaturated step
        double uNextBase = Math.min(1.0, time + (dt / totalTime));
        double sNextBase = smoothstep(uNextBase);
        double sliderNextBase = computeSlider(curve.getPositionAtArcLengthTime(sNextBase));
        double sliderVel_base = (sliderNextBase - sliderUnits) / dt; // distance units / s (at g=1)

        // Build Jacobian at current pose
        double theta1 = Math.toRadians(rawShoulderDeg);
        double theta2 = Math.toRadians(elbowDeg);
        double sEff   = L2 + sliderUnits; // effective forearm

        double s1  = Math.sin(theta1);
        double c1  = Math.cos(theta1);
        double s12 = Math.sin(theta1 + theta2);
        double c12 = Math.cos(theta1 + theta2);

        double J11 = -L1 * s1 - sEff * s12;
        double J12 =        - sEff * s12;
        double J21 =  L1 * c1 + sEff * c12;
        double J22 =          sEff * c12;

        // Subtract slider contribution at base rate (so shoulder/elbow only see what's left)
        double vx_base = dxdT_base - (c12 * sliderVel_base);
        double vy_base = dydT_base - (s12 * sliderVel_base);

        // Solve for joint rates at base uDot
        double det = J11 * J22 - J12 * J21;
        double shoulderVelDeg_raw;
        double elbowVelDeg_raw;
        if (Math.abs(det) > 1e-7) {
            double invDet   = 1.0 / det;
            double theta1Dt =  invDet * ( J22 * vx_base - J12 * vy_base );
            double theta2Dt =  invDet * (-J21 * vx_base + J11 * vy_base );
            shoulderVelDeg_raw = Math.toDegrees(theta1Dt);
            elbowVelDeg_raw    = Math.toDegrees(theta2Dt);
        } else {
            // singular fallback
            shoulderVelDeg_raw = (rawShoulderDeg - lastRawShoulderDeg) / dt;
            elbowVelDeg_raw    = (elbowDeg       - lastElbowDeg      ) / dt;
        }

        double sliderRPS_raw = sliderVel_base / SLIDER_UNITS_PER_REV;

        // === 2) Retiming: compute global scale g so no axis exceeds limits ===
        double g = 1.0;
        g = Math.min(g, safeScale(shoulderVelDeg_raw, MAX_SHOULDER_VEL));
        g = Math.min(g, safeScale(elbowVelDeg_raw,    MAX_ELBOW_VEL   ));
        g = Math.min(g, safeScale(sliderRPS_raw,      MAX_SLIDER_RPS ));

        // === 3) Command scaled velocities ===
        double shoulderVelCmd = shoulderVelDeg_raw * g;
        double elbowVelCmd    = elbowVelDeg_raw    * g * ELBOW_SIGN; // apply observed sign
        double sliderRPSCmd   = sliderRPS_raw      * g;

        // Apply deadbands to kill numeric creep
        shoulderVelCmd = applyDeadband(shoulderVelCmd, VEL_DB_SHOULDER);
        elbowVelCmd    = applyDeadband(elbowVelCmd,    VEL_DB_ELBOW);
        sliderRPSCmd   = applyDeadband(sliderRPSCmd,   VEL_DB_SLIDER);

        // Stop window near the end (use final IK) — ensures hard zero
        double shErr = rawShoulderDeg - finalRawShoulderDeg;
        double elErr = elbowDeg       - finalElbowDeg;
        double slErr = sliderUnits    - finalSliderUnits;
        if ((Math.abs(shErr) < ANG_TOL_DEG && Math.abs(elErr) < ANG_TOL_DEG && Math.abs(slErr) < SLIDER_TOL
             && (sp < SP_TOL || time > 0.999)) ) {
            shoulderVelCmd = 0; elbowVelCmd = 0; sliderRPSCmd = 0;
            time = 1.0; // force completion
        }

        // Dashboard (angles clamped for display only)
        SmartDashboard.putNumber("ShoulderDeg",   clamp(rawShoulderDeg, -180.0, MAX_SHOULDER_DEG));
        SmartDashboard.putNumber("ElbowDeg",      elbowDeg);
        SmartDashboard.putNumber("CurrentSlider", sliderUnits);
        SmartDashboard.putNumber("ShoulderVelCmd", shoulderVelCmd);
        SmartDashboard.putNumber("ElbowVelCmd",     elbowVelCmd);
        SmartDashboard.putNumber("SliderRPSCmd",    sliderRPSCmd);
        SmartDashboard.putNumber("time", time);
        SmartDashboard.putNumber("g_scale", g);
        SmartDashboard.putNumber("sp", sp);
        SmartDashboard.putNumber("shErr", shErr);
        SmartDashboard.putNumber("elErr", elErr);
        SmartDashboard.putNumber("slErr", slErr);

        arm.setJointVelocities(shoulderVelCmd, elbowVelCmd, sliderRPSCmd);

        // progress advances slower if we had to downscale
        time = Math.min(1.0, time + (g * dt / totalTime));

        // update history
        lastRawShoulderDeg = rawShoulderDeg;
        lastElbowDeg       = elbowDeg;
        lastSliderUnits    = sliderUnits;
    }
    @Override
    public boolean isFinished() {
        return time >= totalTime;
    }

    @Override
    public void end(boolean interrupted) {
arm.setJointAngles(rawShoulderDeg, elbowDeg,sliderUnits);

        //arm.setJointVelocities(0, 0, 0);
    }

    // —— helpers ——

    private static double smoothstep(double u) {
        return u*u*(3 - 2*u);
    }
    private static double smoothstepDeriv(double u) {
        return 6*u*(1 - u);
    }

    /** Unclamped shoulder angle (deg) from base & target */
    private double computeRawShoulderDeg(Point2D.Double pos) {
        double dx   = pos.x - base.x;
        double dy   = pos.y - base.y;
        double dist = Math.hypot(dx, dy);
        double slider = clamp(dist - L1 - L2, 0, MAX_SLIDER_LENGTH);
        double sEff   = L2 + slider;

        double baseAng = Math.atan2(dy, dx);
        double cosArg  = clamp((L1*L1 + dist*dist - sEff*sEff)/(2*L1*dist), -1, 1);
        double theta1  = Math.acos(cosArg);
        return Math.toDegrees(baseAng - theta1);
    }

    /** Interior elbow angle (deg) between L1 and (L2+slider) */
    private double computeElbowDeg(Point2D.Double pos) {
        double dx   = pos.x - base.x;
        double dy   = pos.y - base.y;
        double dist = Math.hypot(dx, dy);
        double slider = clamp(dist - L1 - L2, 0, MAX_SLIDER_LENGTH);
        double sEff   = L2 + slider;

        double cos2 = clamp((L1*L1 + sEff*sEff - dist*dist)/(2*L1*sEff), -1, 1);
        return Math.toDegrees(Math.acos(cos2));
    }

    /** Slider extension (same units as L1/L2) */
    private double computeSlider(Point2D.Double pos) {
        double dist = base.distance(pos);
        return clamp(dist - L1 - L2, 0, MAX_SLIDER_LENGTH);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
