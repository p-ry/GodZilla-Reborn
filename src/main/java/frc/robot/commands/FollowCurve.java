package frc.robot.commands;

import java.awt.geom.Point2D;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAssembly;
import frc.robot.BezierCurveJava;

/**
 * Follow a cubic Bezier path with retimed velocity limiting.
 * This version adds **forward kinematics (FK)** computed from **sensor readings**
 * to publish the hardware end-effector (EE) position alongside the target.
 *
 * Elbow sensor is assumed **relative** (measures the hinge interior angle).
 * Units: millimeters for distances; degrees for angles.
 */
public class FollowCurve extends Command {
    private final ArmAssembly arm;
    private final BezierCurveJava curve;
    private final Point2D.Double base; // shoulder joint position in field coords

    // ===== Optional sensor suppliers for FK (minimal integration) =====
    // If these are non-null, FK will be computed from sensors.
    private final DoubleSupplier shoulderDegHwSup; // degrees
    private final DoubleSupplier elbowDegHwSup;    // degrees (relative interior)
    private final DoubleSupplier sliderUnitsHwSup; // rotations or mm depending on flag
    private final boolean sliderIsRotations;       // true if slider supplier returns rotations

    // ===== Timing (u ~ progress 0..1) =====
    private static final double totalTime = 5.0;   // planned time to traverse u from 0→1 when unsaturated
    private static final double dt        = 0.02;  // loop period
    private double time;                           // progress 0..1

    // ===== Arm geometry & limits (use consistent units with your path points) =====
    private static final double L1 = 496.2; // upper arm length (mm)
    private static final double L2 = 696.9; // forearm base length (mm) (no slider)
    private static final double MAX_SHOULDER_DEG = 70.0; // mechanical limit (deg)
    private static final double MAX_SHOULDER_VEL = 40.0; // deg/s command cap
    private static final double MAX_ELBOW_VEL    = 20.0; // deg/s command cap
    private static final double MAX_SLIDER_LENGTH= 350.0; // slider stroke (mm)
    private static final double MAX_SLIDER_RPS   = 1.0;  // command cap
    private static final double SLIDER_UNITS_PER_REV = 142.875; // mm per rev (ensure correct)

    // ===== Direction & smoothing =====
    private static final double ELBOW_SIGN = -1.0;       // invert elbow command if needed
    private static final double VEL_DB_SHOULDER = 0.05;  // deg/s deadband
    private static final double VEL_DB_ELBOW    = 0.05;  // deg/s deadband
    private static final double VEL_DB_SLIDER   = 0.005; // rps deadband
    private static final double ANG_TOL_DEG     = 0.25;  // stop window (angles)
    private static final double SLIDER_TOL      = 0.5;   // stop window (mm)
    private static final double SP_TOL          = 1e-3;  // smoothstep' threshold to call it "at end"

    // ===== Final target (for stop window / creep kill) =====
    private Point2D.Double finalPos;
    private double finalRawShoulderDeg;
    private double finalElbowDeg;
    private double finalSliderUnits;

    // ===== History for singular fallback =====
    private double lastRawShoulderDeg;
    private double lastElbowDeg;
    private double lastSliderUnits;



    // ===== Mapping HW sensors -> IK angles (calibrated at start pose) =====
    private static final double SHOULDER_SIGN = +1.0; // flip if your sensor increases opposite to IK frame
    private static final double ELBOW_MEAS_SIGN = +1.0; // sign for relative elbow sensor
    private double shoulderOffsetIK = 0.0; // deg; maps (SHOULDER_SIGN*HW) - offset -> IK θ1
    private double elbowOffsetIK    = 0.0; // deg; maps (ELBOW_MEAS_SIGN*HW) - offset -> IK θ2 (interior)
    private double sliderZeroMM     = 0.0; // mm; maps HW slider to mm extension

    // ===== Constructors =====
    public FollowCurve(
            ArmAssembly arm,
            Point2D.Double p0, Point2D.Double p1,
            Point2D.Double p2, Point2D.Double p3,
            Point2D.Double base) {
        this(arm, p0, p1, p2, p3, base, null, null, null, true);
    }

    /**
     * Constructor with sensor suppliers for FK (non-invasive to ArmAssembly API).
     * @param shoulderDegHw supplier of shoulder angle in degrees (hardware reading)
     * @param elbowDegHw supplier of elbow interior angle in degrees (relative sensor)
     * @param sliderUnitsHw supplier of slider sensor units (rotations if sliderIsRotations==true, else mm)
     * @param sliderIsRotations true if slider supplier returns rotations
     */
    public FollowCurve(
            ArmAssembly arm,
            Point2D.Double p0, Point2D.Double p1,
            Point2D.Double p2, Point2D.Double p3,
            Point2D.Double base,
            DoubleSupplier shoulderDegHw,
            DoubleSupplier elbowDegHw,
            DoubleSupplier sliderUnitsHw,
            boolean sliderIsRotations) {
        this.arm = arm;
        this.curve = new BezierCurveJava(p0, p1, p2, p3);
        this.base = base;
        this.shoulderDegHwSup = shoulderDegHw;
        this.elbowDegHwSup = elbowDegHw;
        this.sliderUnitsHwSup = sliderUnitsHw;
        this.sliderIsRotations = sliderIsRotations;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        time = 0.0;
        Point2D.Double startPos = curve.getPositionAtArcLengthTime(0.0);
        lastRawShoulderDeg = computeRawShoulderDeg(startPos);
        lastElbowDeg       = computeElbowDeg(startPos);
        lastSliderUnits    = computeSlider(startPos);

        // Precompute terminal pose/angles for a crisp stop
        finalPos             = curve.getPositionAtArcLengthTime(1.0);
        finalRawShoulderDeg  = computeRawShoulderDeg(finalPos);
        finalElbowDeg        = computeElbowDeg(finalPos);
        finalSliderUnits     = computeSlider(finalPos);

        // ===== Calibrate HW->IK mapping at the start pose (if suppliers present) =====
        if (shoulderDegHwSup != null && elbowDegHwSup != null && sliderUnitsHwSup != null) {
            double startIK_theta1 = lastRawShoulderDeg; // IK shoulder at startPos
            double startIK_theta2 = lastElbowDeg;       // IK elbow at startPos (interior)
            double startIK_slider = lastSliderUnits;    // IK slider mm at startPos

            double hwSh = shoulderDegHwSup.getAsDouble();
            double hwEl = elbowDegHwSup.getAsDouble();
            double hwSl = sliderUnitsHwSup.getAsDouble();

            shoulderOffsetIK = SHOULDER_SIGN * hwSh - startIK_theta1;
            elbowOffsetIK    = ELBOW_MEAS_SIGN * hwEl - startIK_theta2; // relative sensor already interior angle
            double hwSliderMM = sliderIsRotations ? hwSl * SLIDER_UNITS_PER_REV : hwSl;
            sliderZeroMM      = hwSliderMM - startIK_slider;

            SmartDashboard.putBoolean("FK_Calibrated", true);
        } else {
            SmartDashboard.putBoolean("FK_Calibrated", false);
        }
    }

    @Override
    public void execute() {
        if (time >= 1.0) { arm.setJointVelocities(0, 0, 0); return; }

        // === 1) Time-warp and sampling ===
        double s  = smoothstep(time);           // position progress (arc-length parameter)
        double sp = smoothstepDeriv(time);      // ds/du (zero at ends)

        Point2D.Double pos     = curve.getPositionAtArcLengthTime(s);
        Point2D.Double dPos_ds = curve.getVelocityAtArcLengthTime(s); // ∂pos/∂s

        // Base (unsaturated) Cartesian velocity from uDot = 1/totalTime
        double dxdT_base = dPos_ds.x * (sp / totalTime);
        double dydT_base = dPos_ds.y * (sp / totalTime);

        // Current kinematics at pose (angles depend only on pos)
        double rawShoulderDeg = computeRawShoulderDeg(pos);
        double elbowDeg       = computeElbowDeg(pos);
        double sliderUnits    = computeSlider(pos);

        // Predict slider rate for base uDot using finite diff over one unsaturated step
        double uNextBase      = Math.min(1.0, time + (dt / totalTime));
        double sNextBase      = smoothstep(uNextBase);
        double sliderNextBase = computeSlider(curve.getPositionAtArcLengthTime(sNextBase));
        double sliderVel_base = (sliderNextBase - sliderUnits) / dt; // mm/s (at g=1)

        // Build Jacobian at current pose
        double theta1 = Math.toRadians(rawShoulderDeg);
        double theta2 = Math.toRadians(elbowDeg);
        double sEff   = L2 + sliderUnits; // effective forearm (mm)

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

        double sliderRPS_raw = sliderVel_base / SLIDER_UNITS_PER_REV; // rps at g=1

        // === 2) Retiming: compute global scale g so no axis exceeds limits ===
        double g = 1.0;
        g = Math.min(g, safeScale(shoulderVelDeg_raw, MAX_SHOULDER_VEL));
        g = Math.min(g, safeScale(elbowVelDeg_raw,    MAX_ELBOW_VEL   ));
        g = Math.min(g, safeScale(sliderRPS_raw,      MAX_SLIDER_RPS  ));

        // === 3) Command scaled velocities ===
        double shoulderVelCmd = applyDeadband(shoulderVelDeg_raw * g, VEL_DB_SHOULDER);
        double elbowVelCmd    = applyDeadband(elbowVelDeg_raw    * g * ELBOW_SIGN, VEL_DB_ELBOW);
        double sliderRPSCmd   = applyDeadband(sliderRPS_raw      * g, VEL_DB_SLIDER);

        // REPLACE your nearEnd line with this:
boolean nearEnd = (time > 0.95) || (sp < SP_TOL && time > 0.05);
// Stop window near the end — use HARDWARE ANGLES; slider not required
double shErr = rawShoulderDeg - finalRawShoulderDeg;  // IK fallback
double elErr = elbowDeg       - finalElbowDeg;

// Map sensors → IK frame if available and use those errors instead
boolean usingHW = false;
if (shoulderDegHwSup != null && elbowDegHwSup != null) {
    double hwSh_deg = shoulderDegHwSup.getAsDouble();
    double hwEl_deg = elbowDegHwSup.getAsDouble();
    double theta1IK_deg = SHOULDER_SIGN   * hwSh_deg - shoulderOffsetIK; // shoulder world angle in IK frame
    double theta2IK_deg = ELBOW_MEAS_SIGN * hwEl_deg - elbowOffsetIK;    // elbow interior in IK frame
    shErr = theta1IK_deg - finalRawShoulderDeg;
    elErr = theta2IK_deg - finalElbowDeg;
    usingHW = true;
}

// only angles; slider may be in different units, so don’t gate on it
boolean anglesGood = Math.abs(shErr) < ANG_TOL_DEG && Math.abs(elErr) < ANG_TOL_DEG;

// use your nearEnd flag (so we don’t stop at the start where sp≈0)
if (nearEnd && anglesGood) {
    shoulderVelCmd = 0;
    elbowVelCmd    = 0;
    sliderRPSCmd   = 0;
    time = 1.0; // force completion
}

// debug (optional)
SmartDashboard.putBoolean("Stop_UsingHW", usingHW);
SmartDashboard.putNumber("Stop_shErr_deg", shErr);
SmartDashboard.putNumber("Stop_elErr_deg", elErr);
SmartDashboard.putBoolean("nearEnd", nearEnd);


        // Command
        arm.setJointVelocities(shoulderVelCmd, elbowVelCmd, sliderRPSCmd);

        // === 4) Progress advances slower if we had to downscale ===
        time = Math.min(1.0, time + (g * dt / totalTime));

        // === 5) Telemetry (angles clamped for display only) ===
        SmartDashboard.putNumber("ShoulderDeg",   clamp(rawShoulderDeg, -180.0, MAX_SHOULDER_DEG));
        SmartDashboard.putNumber("ElbowDeg",      elbowDeg);
        SmartDashboard.putNumber("CurrentSlider", sliderUnits);
        SmartDashboard.putNumber("ShoulderVelCmd", shoulderVelCmd);
        SmartDashboard.putNumber("ElbowVelCmd",     elbowVelCmd);
        SmartDashboard.putNumber("SliderRPSCmd",    sliderRPSCmd);
        SmartDashboard.putNumber("u", time);
        SmartDashboard.putNumber("g_scale", g);
        SmartDashboard.putNumber("sp", sp);
        SmartDashboard.putNumber("shErr", shErr);
        SmartDashboard.putNumber("elErr", elErr);
        

        // ===== NEW: Forward Kinematics from sensors (if available) =====
        if (shoulderDegHwSup != null && elbowDegHwSup != null && sliderUnitsHwSup != null) {
            double hwSh_deg = shoulderDegHwSup.getAsDouble();
            double hwEl_deg = elbowDegHwSup.getAsDouble();
            double hwSl     = sliderUnitsHwSup.getAsDouble(); // rotations or mm

            // Map sensors -> IK frame
            double theta1IK_deg = SHOULDER_SIGN    * hwSh_deg - shoulderOffsetIK;     // shoulder world angle
            double theta2IK_deg = ELBOW_MEAS_SIGN  * hwEl_deg - elbowOffsetIK;        // elbow interior angle
            double sliderMM     = (sliderIsRotations ? hwSl * SLIDER_UNITS_PER_REV : hwSl) - sliderZeroMM;

            // FK
            double th1 = Math.toRadians(theta1IK_deg);
            double th2 = Math.toRadians(theta2IK_deg);
            double sEffMM = L2 + sliderMM;
            double xHW = base.x + L1 * Math.cos(th1) + sEffMM * Math.cos(th1 + th2);
            double yHW = base.y + L1 * Math.sin(th1) + sEffMM * Math.sin(th1 + th2);

            // Telemetry: target vs hardware EE
            SmartDashboard.putNumber("EE_X_Target", pos.x);
            SmartDashboard.putNumber("EE_Y_Target", pos.y);
            SmartDashboard.putNumber("EE_X_HW",     xHW);
            SmartDashboard.putNumber("EE_Y_HW",     yHW);
            SmartDashboard.putNumber("ShoulderDeg_HW", hwSh_deg);
            SmartDashboard.putNumber("ElbowDeg_HW",    hwEl_deg);
            SmartDashboard.putNumber("SliderMM_HW",    sliderMM);
            SmartDashboard.putNumber("ShoulderDeg_HW_asIK", theta1IK_deg);
            SmartDashboard.putNumber("ElbowDeg_HW_asIK",    theta2IK_deg);
        }

        // update history
        lastRawShoulderDeg = rawShoulderDeg;
        lastElbowDeg       = elbowDeg;
        lastSliderUnits    = sliderUnits;
    }

    @Override
    public boolean isFinished() { 
        

        
        
        return time >= 1.0; }

    @Override
    public void end(boolean interrupted) { arm.setJointVelocities(0, 0, 0); 
        arm.lowerArm.setPos(lastRawShoulderDeg);   
        arm.upperArm.setPos(lastElbowDeg); // interior angle
        arm.slider.setPos(lastSliderUnits);
    }

    // ===== Helpers =====
    private static double smoothstep(double u) { return u*u*(3 - 2*u); }
    private static double smoothstepDeriv(double u) { return 6*u*(1 - u); }

    private static double safeScale(double value, double maxAbs) {
        double a = Math.abs(value);
        if (a < 1e-6) return 1.0;         // nothing to scale
        return Math.min(1.0, maxAbs / a); // ≤ 1.0
    }

    private static double applyDeadband(double v, double db) {
        return (Math.abs(v) < db) ? 0.0 : v;
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

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

    /** Slider extension (mm) */
    private double computeSlider(Point2D.Double pos) {
        double dist = base.distance(pos);
        return clamp(dist - L1 - L2, 0, MAX_SLIDER_LENGTH);
    }
}
