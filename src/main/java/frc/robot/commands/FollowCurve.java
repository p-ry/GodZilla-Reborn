package frc.robot.commands;

import java.awt.geom.Point2D;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAssembly;
import frc.robot.BezierCurveJava;

/**
 * Position-only Bezier follower.
 *
 * Each loop we:
 *  1) advance a normalized progress "u" from 0→1 over totalTime (with smoothstep easing)
 *  2) sample the Bezier at s = smoothstep(u)
 *  3) convert that (x,y) to joint targets (shoulder deg, elbow interior deg, slider mm)
 *  4) command joint *positions* directly (no velocity commands, no Jacobian)
 *
 * Elbow sensor is relative (hinge interior). Units: mm & deg.
 */
public class FollowCurve extends Command {
    private final ArmAssembly arm;
    private final BezierCurveJava curve;
    private final Point2D.Double base; // shoulder joint position in field coords

    // Optional suppliers retained for telemetry only (not used for control)
    private final DoubleSupplier shoulderDegHwSup; // degrees
    private final DoubleSupplier elbowDegHwSup;    // degrees (relative interior)
    private final DoubleSupplier sliderUnitsHwSup; // rotations or mm depending on flag
    private final boolean sliderIsRotations;       // true if slider supplier returns rotations

    // ===== Timing (u ~ progress 0..1) =====
    private static final double totalTime = 2.5; // planned time to traverse u 0→1
    private static final double dt        = 0.02; // loop period
    private double time;                         // progress 0..1

    // ===== Arm geometry & limits (use consistent units with your path points) =====
    private static final double L1 = 496.2; // upper arm length (mm)
    private static final double L2 = 696.9; // forearm base length (mm) (no slider)
    private static final double MAX_SLIDER_LENGTH = 350.0; // slider stroke (mm)
    private static final double SLIDER_UNITS_PER_REV = 142.875; // mm per rev (telemetry only)

    // ===== Final target (for end/hold) =====
    private Point2D.Double finalPos;
    private double finalRawShoulderDeg;
    private double finalElbowDeg;
    private double finalSliderUnits;

    // ===== Last commanded (used by your end() to hold) =====
    private double lastRawShoulderDeg;
    private double lastElbowDeg;
    private double lastSliderUnits;

    public FollowCurve(
            ArmAssembly arm,
            Point2D.Double p0, Point2D.Double p1,
            Point2D.Double p2, Point2D.Double p3,
            Point2D.Double base) {
        this(arm, p0, p1, p2, p3, base, null, null, null, true);
    }

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

        // Precompute terminal pose/angles
        finalPos             = curve.getPositionAtArcLengthTime(1.0);
        finalRawShoulderDeg  = computeRawShoulderDeg(finalPos);
        finalElbowDeg        = computeElbowDeg(finalPos);
        finalSliderUnits     = computeSlider(finalPos);

        SmartDashboard.putBoolean("FK_Calibrated", (shoulderDegHwSup!=null && elbowDegHwSup!=null && sliderUnitsHwSup!=null));
    }

    @Override
    public void execute() {
        if (time >= 1.0) {
            // Re-command final pose to be explicit, then finish
            arm.lowerArm.setPos(finalRawShoulderDeg);
            arm.upperArm.setPos(finalElbowDeg);
            arm.slider.setPos(finalSliderUnits);
            return;
        }

        // 1) Time-warp and sampling
        double s  = smoothstep(time);           // eased progress 0..1
        Point2D.Double pos = curve.getPositionAtArcLengthTime(s);

        // 2) IK to joint targets (deg, deg, mm)
        double rawShoulderDeg = computeRawShoulderDeg(pos); // world shoulder angle (deg)
        double elbowDeg       = computeElbowDeg(pos);       // interior elbow angle (deg)
        double sliderUnits    = computeSlider(pos);         // extension (mm)

        // 3) Command *positions* directly
        arm.lowerArm.setPos(rawShoulderDeg);
        arm.upperArm.setPos(elbowDeg);
        arm.slider.setPos(sliderUnits);

        // 4) Advance progress at constant rate over totalTime
        time = Math.min(1.0, time + (dt / totalTime));

        // 5) Telemetry
        SmartDashboard.putNumber("u", time);
        SmartDashboard.putNumber("Bezier_s", s);
        SmartDashboard.putNumber("Target_ShoulderDeg", rawShoulderDeg);
        SmartDashboard.putNumber("Target_ElbowDeg", elbowDeg);
        SmartDashboard.putNumber("Target_SliderMM", sliderUnits);
        SmartDashboard.putNumber("Final_ShoulderDeg", finalRawShoulderDeg);
        SmartDashboard.putNumber("Final_ElbowDeg", finalElbowDeg);
        SmartDashboard.putNumber("Final_SliderMM", finalSliderUnits);

        // Optional HW telemetry if suppliers provided
        if (shoulderDegHwSup != null) SmartDashboard.putNumber("HW_ShoulderDeg", shoulderDegHwSup.getAsDouble());
        if (elbowDegHwSup    != null) SmartDashboard.putNumber("HW_ElbowDeg",    elbowDegHwSup.getAsDouble());
        if (sliderUnitsHwSup != null) {
            double hw = sliderUnitsHwSup.getAsDouble();
            double mm = sliderIsRotations ? hw * SLIDER_UNITS_PER_REV : hw;
            SmartDashboard.putNumber("HW_SliderMM", mm);
        }

        // Update last for your end() hold
        lastRawShoulderDeg = rawShoulderDeg;
        lastElbowDeg       = elbowDeg;
        lastSliderUnits    = sliderUnits;
    }

    @Override
    public boolean isFinished() { return time >= 1.0; }

    // DO NOT MODIFY: you asked to keep your end() behavior for hold
    @Override
    public void end(boolean interrupted) {
        //arm.setJointVelocities(0, 0, 0); // harmless even though we don't use velocities now
        arm.lowerArm.setPos(lastRawShoulderDeg);
        arm.upperArm.setPos(lastElbowDeg); // interior angle
        arm.slider.setPos(lastSliderUnits);
    }

    // ===== Helpers =====
    private static double smoothstep(double u) { return u*u*(3 - 2*u); }

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

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
