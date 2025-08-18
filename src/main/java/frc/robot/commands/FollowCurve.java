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
        if (time > totalTime) {
            arm.setJointVelocities(0, 0, 0);
            return;
        }

        // 1) smoothstep time warp → zero start/stop velocity
        double u   = time / totalTime;
        double s   = smoothstep(u);
        double sp  = smoothstepDeriv(u);

        // 2) sample curve at arc-length param s
        Point2D.Double pos = curve.getPositionAtArcLengthTime(s);
        Point2D.Double vel = curve.getVelocityAtArcLengthTime(s);

        // 3) world-frame velocity dpos/dt
        double scale = sp / totalTime;
        double dxdT  = vel.x * scale;
        double dydT  = vel.y * scale;

        // 4) kinematics at this pose
        rawShoulderDeg = computeRawShoulderDeg(pos);
        elbowDeg       = computeElbowDeg(pos);   // interior L1–(L2+slider)
        sliderUnits    = computeSlider(pos);

        // 5) finite-difference slider velocity (distance units / s)
        double sliderVel = (sliderUnits - lastSliderUnits) / dt;

        // 6) build Jacobian terms
        double theta1 = Math.toRadians(rawShoulderDeg);
        double theta2 = Math.toRadians(elbowDeg);
        double sEff   = L2 + sliderUnits; // effective forearm length

        double s1  = Math.sin(theta1);
        double c1  = Math.cos(theta1);
        double s12 = Math.sin(theta1 + theta2);
        double c12 = Math.cos(theta1 + theta2);

        // 7) 2x2 Jacobian for (theta1, theta2)
        double J11 = -L1 * s1 - sEff * s12;
        double J12 =        - sEff * s12;
        double J21 =  L1 * c1 + sEff * c12;
        double J22 =          sEff * c12;

        // 8) subtract slider contribution to end-effector velocity
        // d/dt (pos) due to slider = [c12, s12] * sliderVel
        double vx = dxdT - (c12 * sliderVel);
        double vy = dydT - (s12 * sliderVel);

        // 9) solve joint rates with singularity fallback
        double det = J11 * J22 - J12 * J21;
        double shoulderVelDeg;
        double elbowVelDeg;
        if (Math.abs(det) > 1e-7) {
            double invDet   = 1.0 / det;
            double theta1Dt =  invDet * ( J22 * vx - J12 * vy );
            double theta2Dt =  invDet * (-J21 * vx + J11 * vy );
            shoulderVelDeg  = Math.toDegrees(theta1Dt);
            elbowVelDeg     = Math.toDegrees(theta2Dt);
        } else {
            // near singular (elbow straight/fully bent) — fall back to finite difference
            shoulderVelDeg  = (rawShoulderDeg - lastRawShoulderDeg) / dt;
            elbowVelDeg     = (elbowDeg       - lastElbowDeg      ) / dt;
        }

        // 10) clamp command velocities
        double shoulderVelCmd = clamp(shoulderVelDeg, -MAX_SHOULDER_VEL, MAX_SHOULDER_VEL);
        double elbowVelCmd    = clamp(elbowVelDeg,    -MAX_ELBOW_VEL,    MAX_ELBOW_VEL);
        double sliderRPSCmd   = clamp(sliderVel / SLIDER_UNITS_PER_REV, -MAX_SLIDER_RPS, MAX_SLIDER_RPS);

        // 11) dashboard (angles are clamped-for-display; velocities are the command values)
        SmartDashboard.putNumber("ShoulderDeg",   clamp(rawShoulderDeg, -180.0, MAX_SHOULDER_DEG));
        SmartDashboard.putNumber("ElbowDeg",      elbowDeg);
        SmartDashboard.putNumber("CurrentSlider", sliderUnits);
        SmartDashboard.putNumber("ShoulderVelCmd", shoulderVelCmd);
        SmartDashboard.putNumber("ElbowVelCmd",     elbowVelCmd);
        SmartDashboard.putNumber("SliderRPSCmd",    sliderRPSCmd);

        // 12) send to hardware
        arm.setJointVelocities(shoulderVelCmd, 0,0);//elbowVelCmd, sliderRPSCmd);

        // 13) update history
        lastRawShoulderDeg = rawShoulderDeg;
        lastElbowDeg       = elbowDeg;
        lastSliderUnits    = sliderUnits;
        time              += dt;
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
