package frc.robot.commands;

import java.util.List;

import com.ctre.phoenix6.configs.Slot2Configs;

import java.awt.geom.Point2D;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import java.awt.geom.GeneralPath;
//import java.awt.geom.Path2D;
//import java.awt.geom.*;

import frc.robot.subsystems.ArmAssembly;
import frc.robot.BezierLogger;
import frc.robot.BezierPlotTool;
import frc.robot.RobotContainer;
import frc.robot.Utilitys.BezierCurve;
import frc.robot.BezierCurveJava;
//import frc.robot.subsystems.ArmController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAssembly;
import frc.robot.Vector2D;

public class FollowCurve extends Command {
    private final ArmAssembly arm;
    private final BezierCurveJava curve;
    private final Point2D.Double base;

    private static final double totalTime = 1.0;
    private static final double dt = 0.02;
    private double time;

    // arm geometry & limits
    private static final double L1 = 496.2; // mm
    private static final double L2 = 696.9; // mm
    private static final double MAX_SHOULDER_DEG = 70.0; // °
    private static final double MAX_SLIDER_LENGTH = 350.0; // mm
    private static final double SLIDER_METERS_PER_REV = 0.142875; // mm per rev
    private double sL2 = 0;

    // state for finite-difference
    private double lastShoulderDeg;
    private double lastElbowDeg;
    private double lastSliderMeters;

    private double shoulderDeg, elbowDeg;

    public FollowCurve(
            ArmAssembly arm,
            Point2D.Double p0, Point2D.Double p1,
            Point2D.Double p2, Point2D.Double p3,
            Point2D.Double base) {
        this.arm = arm;
        this.curve = new BezierCurveJava(p0, p1, p2, p3);
        this.base = base;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        time = 0;
        Point2D.Double startPos = curve.getPositionAtArcLengthTime(0.0);

    // seed so (deg₀ − deg₀)/dt == 0
    lastShoulderDeg  = computeShoulderDeg(startPos);
    lastElbowDeg     = computeElbowDeg(startPos);
    lastSliderMeters = computeSlider(startPos);
    }

    @Override
    public void execute() {
        if (time > totalTime) {
            arm.setJointVelocities(0, 0, 0);
            return;
        }

        // 1) normalize time and calculate smoothstep warp
        double u = time / totalTime;
        double s = smoothstep(u); // maps [0→1] with zero slope at ends
        double sp = smoothstepDeriv(u); // = d s/du

        // 2) sample curve at arc‐length parameter s
        Point2D.Double pos = curve.getPositionAtArcLengthTime(s);
        Point2D.Double vel = curve.getVelocityAtArcLengthTime(s);

        // 3) world‐frame velocity:
        // dpos/dt = dpos/ds * (ds/du) * (du/dt) = vel * sp * (1/totalTime)
        double scale = sp / totalTime;
        double dxdT = vel.x * scale;
        double dydT = vel.y * scale;

        // 4) inverse kinematics for this pos
        shoulderDeg = computeShoulderDeg(pos);
        double elbowDeg = computeElbowDeg(pos);
        double sliderMM = computeSlider(pos);

        // 5) finite-difference joint velocities
        double shoulderVel = (shoulderDeg - lastShoulderDeg) / dt;
        double elbowVel = (elbowDeg - lastElbowDeg) / dt;
        double sliderVelM = (sliderMM - lastSliderMeters) / dt;
        double sliderRPS = sliderVelM / SLIDER_METERS_PER_REV;

        // 6) dashboard & command
        SmartDashboard.putNumber("ShoulderDeg", shoulderDeg);
        SmartDashboard.putNumber("ElbowDeg", elbowDeg);
        SmartDashboard.putNumber("CurrentSlider", sliderMM);

        arm.setJointVelocities(shoulderVel, elbowVel, sliderRPS);

        // 7) remember for next cycle
        lastShoulderDeg = shoulderDeg;
        lastElbowDeg = elbowDeg;
        lastSliderMeters = sliderMM;
        time += dt;
    }

    @Override
    public boolean isFinished() {
        return time >= totalTime;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setJointVelocities(0, 0, 0);
    }

    // ——— helpers ———

    /** Smoothstep [0→1] with zero derivatives at both ends */
    private static double smoothstep(double u) {
        // simple cubic: 3u^2 − 2u^3
        return u * u * (3 - 2 * u);
    }

    /** derivative of smoothstep: d/du[3u^2-2u^3] = 6u(1 − u) */
    private static double smoothstepDeriv(double u) {
        return 6 * u * (1 - u);
    }


    /** 
 * The *true* shoulder joint angle in degrees, unconstrained 
 */
private double computeRawShoulderDeg(Point2D.Double pos) {
    double dx   = pos.x - base.x;
    double dy   = pos.y - base.y;
    double dist = Math.hypot(dx, dy);

    // slider folded into forearm
    double sliderMM = clamp(dist - L1 - L2, 0, MAX_SLIDER_LENGTH);
    double SL2      = L2 + sliderMM;

    double baseAng = Math.atan2(dy, dx);
    double cosArg  = clamp((L1*L1 + dist*dist - SL2*SL2)/(2*L1*dist), -1, 1);
    double theta1  = Math.acos(cosArg);

    // **no clamp here**, convert to degrees
    return Math.toDegrees(baseAng - theta1);
}

/**
 * What you actually show (and/or enforce) on the robot:
 */
private double clampShoulderDeg(double rawDeg) {
    return clamp(rawDeg, -180.0, MAX_SHOULDER_DEG);
}

    /** IK for shoulder angle (°), clamped to your max */
    private double computeShoulderDeg(Point2D.Double pos) {
        double dx = pos.x - base.x;
        double dy = pos.y - base.y;
        double dist = Math.hypot(dx, dy);
        double rawExt = dist - L1 - L2;
        double sliderMM = clamp(rawExt, 0, MAX_SLIDER_LENGTH);

        // 3) now fold slider into the effective forearm length
        sL2 = L2 + sliderMM;

        // law-of-cos + bearing
        double baseAng = Math.atan2(dy, dx);
        // note: slider is folded into L2 here
        double cosArg = clamp((L1 * L1 + dist * dist - L2 * sL2) / (2 * L1 * dist), -1, 1);
        double theta1 = Math.acos(cosArg);
        return Math.toDegrees(clamp(baseAng - theta1, -Math.PI, Math.toRadians(MAX_SHOULDER_DEG)));
    }

    /** IK for elbow interior angle (°) between L1 and L2 */
    private double computeElbowDeg(Point2D.Double pos) {

        double shoulderRad = Math.toRadians(shoulderDeg);

        // 1) elbow joint pos
        double elbowX = base.x + L1 * Math.cos(shoulderRad);
        double elbowY = base.y + L1 * Math.sin(shoulderRad);

        // 2) two vectors
        double ux = base.x - elbowX;
        double uy = base.y - elbowY;
        double vx = pos.x - elbowX;
        double vy = pos.y - elbowY;

        // 3) elbow angle via dot‐product
        double cosElbow = clamp((ux * vx + uy * vy) / (Math.hypot(ux, uy) * Math.hypot(vx, vy)), -1, 1);
        double elbowDeg = Math.toDegrees(Math.acos(cosElbow));
        return elbowDeg;
    }

    /** slider extension (mm) */
    private double computeSlider(Point2D.Double pos) {
        double dx = pos.x - base.x;
        double dy = pos.y - base.y;
        double dist = Math.hypot(dx, dy);
        return clamp(dist - L1 - L2, 0, MAX_SLIDER_LENGTH);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
