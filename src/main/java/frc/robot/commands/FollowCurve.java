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

public class FollowCurve extends Command {
    private final ArmAssembly arm;
    private final BezierCurveJava curve;
    private final Point2D.Double base;

    private static final double totalTime = 5.0;
    private static final double dt = 0.02;
    private double time;
    private boolean holdMode = false; 
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

        double u = time / totalTime;
        double s = smoothstep(u);
        double sp = smoothstepDeriv(u);

        Point2D.Double pos = curve.getPositionAtArcLengthTime(s);
        Point2D.Double vel = curve.getVelocityAtArcLengthTime(s);

        double x = pos.x - base.x;
        double y = pos.y - base.y;
        double dxdT = vel.x * sp / totalTime;
        double dydT = vel.y * sp / totalTime;

        shoulderDeg = computeShoulderDeg(pos);
        elbowDeg    = computeElbowDeg(pos);
        double sliderMM = computeSlider(pos);

        double shoulderVel = (shoulderDeg - lastShoulderDeg) / dt;
        double elbowVel    = (elbowDeg    - lastElbowDeg   ) / dt;
        double sliderVelM  = (sliderMM    - lastSliderMeters) / dt;
        double sliderRPS   = sliderVelM / SLIDER_METERS_PER_REV;

        SmartDashboard.putNumber("ShoulderDeg", shoulderDeg);
        SmartDashboard.putNumber("ElbowDeg", elbowDeg);
        SmartDashboard.putNumber("CurrentSlider", sliderMM);

        boolean finished = time >= totalTime;
        boolean slow     = Math.abs(shoulderVel) < 1.0 && Math.abs(elbowVel) < 1.0;
        // latch once into hold mode to prevent chatter  
        if (!holdMode && finished && slow) {  
            holdMode = true;  
        }  
        if (holdMode) {
            arm.setJointAngles(shoulderDeg, elbowDeg, sliderMM);
        } else {
            arm.setJointVelocities(shoulderVel, elbowVel, sliderRPS);
        }

        lastShoulderDeg  = shoulderDeg;
        lastElbowDeg     = elbowDeg;
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

    private static double smoothstep(double u) {
        return u * u * (3 - 2 * u);
    }

    private static double smoothstepDeriv(double u) {
        return 6 * u * (1 - u);
    }

    private double computeRawShoulderDeg(Point2D.Double pos) {
        double dx   = pos.x - base.x;
        double dy   = pos.y - base.y;
        double dist = Math.hypot(dx, dy);
        double sliderMM = clamp(dist - L1 - L2, 0, MAX_SLIDER_LENGTH);
        double SL2      = L2 + sliderMM;
        double baseAng = Math.atan2(dy, dx);
        double cosArg  = clamp((L1*L1 + dist*dist - SL2*SL2)/(2*L1*dist), -1, 1);
        double theta1  = Math.acos(cosArg);
        return Math.toDegrees(baseAng - theta1);
    }

    private double computeShoulderDeg(Point2D.Double pos) {
        double dx = pos.x - base.x;
        double dy = pos.y - base.y;
        double dist = Math.hypot(dx, dy);
        double rawExt = dist - L1 - L2;
        double sliderMM = clamp(rawExt, 0, MAX_SLIDER_LENGTH);
        sL2 = L2 + sliderMM;
        double baseAng = Math.atan2(dy, dx);
        double cosArg = clamp((L1 * L1 + dist * dist - sL2 * sL2) / (2 * L1 * dist), -1, 1);
        double theta1 = Math.acos(cosArg);
        return Math.toDegrees(clamp(baseAng - theta1, -Math.PI, Math.toRadians(MAX_SHOULDER_DEG)));
    }

    private double computeElbowDeg(Point2D.Double pos) {
        double shoulderRad = Math.toRadians(shoulderDeg);
        double elbowX = base.x + L1 * Math.cos(shoulderRad);
        double elbowY = base.y + L1 * Math.sin(shoulderRad);
        double ux = base.x - elbowX;
        double uy = base.y - elbowY;
        double vx = pos.x - elbowX;
        double vy = pos.y - elbowY;
        double cosElbow = clamp((ux * vx + uy * vy) /
            (Math.hypot(ux, uy) * Math.hypot(vx, vy)), -1, 1);
        return Math.toDegrees(Math.acos(cosElbow));
    }

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
