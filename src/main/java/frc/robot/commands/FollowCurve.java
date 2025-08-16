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

    // arm geometry & limits
    private static final double L1                 = 496.2;    // mm
    private static final double L2                 = 696.9;    // mm
    private static final double MAX_SHOULDER_DEG   = 70.0;     // °
    private static final double MAX_SHOULDER_VEL   = 2.0;      // deg/s
    private static final double MAX_ELBOW_VEL      = 2.0;      // deg/s
    private static final double MAX_SLIDER_LENGTH  = 350.0;    // mm
    private static final double MAX_SLIDER_RPS     = 1.0;      // rev/s
    private static final double SLIDER_METERS_PER_REV = 0.142875; // mm per rev

    // state for finite-difference
    private double lastRawShoulderDeg;
    private double lastElbowDeg;
    private double lastSliderMeters;
    private boolean holdMode = false; 

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
        lastSliderMeters    = computeSlider(startPos);
    }

    @Override
    public void execute() {
        if (time > totalTime) {
            arm.setJointVelocities(0, 0, 0);
            return;
        }

        // 1) smoothstep time warp
        double u   = time / totalTime;
        double s   = smoothstep(u);
        double sp  = smoothstepDeriv(u);

        // 2) sample curve
        Point2D.Double pos = curve.getPositionAtArcLengthTime(s);
        Point2D.Double vel = curve.getVelocityAtArcLengthTime(s);

        // 3) world-frame velocity
        double scale = sp / totalTime;
        double dxdT  = vel.x * scale;
        double dydT  = vel.y * scale;

        // 4) kinematics
        double shoulderDeg = computeRawShoulderDeg(pos);
        double elbowDeg       = computeElbowDeg(pos);
        double sliderMM       = computeSlider(pos);

        // 5) slider velocity
        double sliderVelM = (sliderMM - lastSliderMeters) / dt;

        // 6) Jacobian geometry
        double theta1Rad = Math.toRadians(shoulderDeg);
        double theta2Rad = Math.toRadians(elbowDeg);
        double sEff      = L2 + sliderMM;
        double s12       = Math.sin(theta1Rad + theta2Rad);
        double c12       = Math.cos(theta1Rad + theta2Rad);
        double s1        = Math.sin(theta1Rad);
        double c1        = Math.cos(theta1Rad);

        // 7) Jacobian entries
        double J11 = -L1 * s1 - sEff * s12;
        double J12 =        - sEff * s12;
        double J21 =  L1 * c1 + sEff * c12;
        double J22 =          sEff * c12;

        // 8) subtract slider effect
        double vx = dxdT - (c12 * sliderVelM);
        double vy = dydT - (s12 * sliderVelM);

        // 9) solve joint rates
        double det    = J11 * J22 - J12 * J21;
        double invDet = 1.0 / det;
        double theta1Dot =  invDet * ( J22 * vx - J12 * vy );
        double theta2Dot =  invDet * (-J21 * vx + J11 * vy );

        // 10) convert and clamp velocities
        double shoulderVel = clamp(Math.toDegrees(theta1Dot), -MAX_SHOULDER_VEL, MAX_SHOULDER_VEL);
        double elbowVel    = clamp(Math.toDegrees(theta2Dot),   -MAX_ELBOW_VEL,    MAX_ELBOW_VEL);
        double sliderRPS   = clamp(sliderVelM / SLIDER_METERS_PER_REV,
                                  -MAX_SLIDER_RPS, MAX_SLIDER_RPS);

        // 11) dashboard & command
        SmartDashboard.putNumber("ShoulderDeg",  clamp(shoulderDeg, -180.0, MAX_SHOULDER_DEG));
        SmartDashboard.putNumber("ElbowDeg",     elbowDeg);
        SmartDashboard.putNumber("CurrentSlider", sliderMM);

        arm.setJointVelocities(shoulderVel, elbowVel, sliderRPS);

         // latch once into hold mode to prevent chatter  
         boolean finished = time >= totalTime;  
         boolean slow     = Math.abs(shoulderVel) < 1.0 && Math.abs(elbowVel) < 1.0;
         if (!holdMode && finished) {  
            holdMode = true;  
        }  
        if (holdMode) {
            arm.setJointAngles(shoulderDeg, elbowDeg, sliderMM);
        } else {
            arm.setJointVelocities(shoulderVel, elbowVel, sliderRPS);
        }
        // 12) update state
        lastRawShoulderDeg = shoulderDeg;
        lastElbowDeg       = elbowDeg;
        lastSliderMeters   = sliderMM;
        time              += dt;
    }

    @Override
    public boolean isFinished() {
        return time >= totalTime;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setJointVelocities(0, 0, 0);
    }

    // —— helpers ——
    private static double smoothstep(double u) {
        return u*u*(3 - 2*u);
    }
    private static double smoothstepDeriv(double u) {
        return 6*u*(1 - u);
    }
    private double computeRawShoulderDeg(Point2D.Double pos) {
        double dx     = pos.x - base.x;
        double dy     = pos.y - base.y;
        double dist   = Math.hypot(dx, dy);
        double slider = clamp(dist - L1 - L2, 0, MAX_SLIDER_LENGTH);
        double sEff   = L2 + slider;

        double baseAng = Math.atan2(dy, dx);
        double cosArg = clamp((L1*L1 + dist*dist - sEff*sEff)/(2*L1*dist), -1, 1);
        double theta1 = Math.acos(cosArg);
        return Math.toDegrees(baseAng - theta1);
    }
    private double computeElbowDeg(Point2D.Double pos) {
        double dx   = pos.x - base.x;
        double dy   = pos.y - base.y;
        double dist = Math.hypot(dx, dy);
        double slider = clamp(dist - L1 - L2, 0, MAX_SLIDER_LENGTH);
        double sEff   = L2 + slider;
        double cos2   = clamp((L1*L1 + sEff*sEff - dist*dist)/(2*L1*sEff), -1, 1);
        return Math.toDegrees(Math.acos(cos2));
    }
    private double computeSlider(Point2D.Double pos) {
        double dist = base.distance(pos);
        return clamp(dist - L1 - L2, 0, MAX_SLIDER_LENGTH);
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
