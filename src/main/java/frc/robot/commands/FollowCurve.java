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
import frc.robot.RobotContainer;
import frc.robot.Utilitys.BezierCurve;
import frc.robot.BezierCurveJava;
//import frc.robot.subsystems.ArmController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAssembly;
import frc.robot.Vector2D;

public class FollowCurve extends Command {
    private final ArmAssembly arm;
    // private final BezierCurve curve;
    private static final double totalTime = 5.0;
    private static final double dt = 0.02;
    private double time;
    private double lastShoulderDeg;
    private double lastSliderMeters;

    private static final double MAX_SHOULDER_DEG = 70.0;
    private static final double MAX_DELTA_SHOULDER_DEG = 4.0;
    private static final double L1 = 496.2; // m
    private static final double L2 = 696.9; // m
    private static final double MAX_SHOULDER_VEL = 2.0; // deg/s
    private static final double MAX_ELBOW_VEL = 2.0; // deg/s
    private static final double MAX_SLIDER_VEL = 0.142875; // m/s
    private static final double MAX_SLIDER_RPS = 1.0; // rps
    private Point2D.Double p0;
    private static final double SLIDER_METERS_PER_REV = 0.142875;
    private static final double MAX_SLIDER_LENGTH = 350.0; // Maximum slider extension (in meters)
    private double lastElbowDeg;

    private Point2D.Double p1;
    private Point2D.Double p2;
    private Point2D.Double p3;
    private Point2D.Double pos;
    private Point2D.Double vel;
    private int numberOfPoints = 40;
    private static List<Point2D.Double> path;
    // private double baseX = 0.0; // Base X coordinate
    // private double baseY = 0.0; // Base Y coordinate
    private Point2D base;
    private BezierCurveJava curve;

    public FollowCurve(ArmAssembly arm, Point2D.Double p0, Point2D.Double p1, Point2D.Double p2, Point2D.Double p3,
            Point2D.Double base) {
        this.arm = arm;

        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        this.base = base;
        System.out.println("p0 = " + p0);
        System.out.println("p1 = " + p1);
        System.out.println("p2 = " + p2);
        System.out.println("p3 = " + p3);

        this.curve = new BezierCurveJava(p0, p1, p2, p3);
        this.time = 0;
        this.lastShoulderDeg = Double.NaN;
        this.lastSliderMeters = 0;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        time = 0;
        lastShoulderDeg = Double.NaN;
        lastSliderMeters = 0;
        lastElbowDeg = Double.NaN;
    }

        @Override
        public void execute() {
            if (time > totalTime) return;
        
            double t = time / totalTime;
        
            // Get position and velocity from Bézier
            Point2D.Double pos = curve.getPositionAtTime(t);
            Point2D.Double vel = curve.getVelocityAtTime(t);
            double dxdT = vel.getX() / totalTime;
            double dydT = vel.getY() / totalTime;
        
            double targetX = pos.getX();
            double targetY = pos.getY();
        
            double dx = targetX - base.getX();
            double dy = targetY - base.getY();
        
            double dist = Math.hypot(dx, dy);
            if (dist > L1 + L2 + MAX_SLIDER_LENGTH || dist < Math.abs(L1 - L2)) {
                SmartDashboard.putString("Unreachable", "Unreachable");
                return; // unreachable
            }
        
            // Initial IK for position
            double currentSlider = Math.max(0, Math.min(dist - L2, MAX_SLIDER_LENGTH));
            double SL2 = L2 + currentSlider;
            double baseAngle = Math.atan2(dy, dx);
            double theta1 = Math.acos(Math.max(-1.0, Math.min(1.0, (L1 * L1 + dist * dist - SL2 * SL2) / (2.0 * L1 * dist))));
            double shoulderAngle = baseAngle - theta1;
            double shoulderDeg = Math.toDegrees(shoulderAngle);
            shoulderDeg = Math.min(shoulderDeg, MAX_SHOULDER_DEG);
        
            // Forward kinematics to get elbow position
            double theta1Rad = Math.toRadians(shoulderDeg);
            double elbowX = base.getX() + L1 * Math.cos(theta1Rad);
            double elbowY = base.getY() + L1 * Math.sin(theta1Rad);
            double tdist = Math.hypot(targetX - elbowX, targetY - elbowY);
        
            currentSlider = Math.max(0, Math.min(tdist - L2, MAX_SLIDER_LENGTH));
            double theta2 = Math.acos(Math.max(-1.0, Math.min(1.0, (L1 * L1 + tdist * tdist - dist * dist) / (2 * L1 * tdist))));
            double elbowDeg = Math.toDegrees(theta2);
        
            // Jacobian-based inverse velocity kinematics
            double theta1RadActual = Math.toRadians(shoulderDeg);
            double theta2RadActual = Math.toRadians(elbowDeg);
            double s1 = Math.sin(theta1RadActual);
            double c1 = Math.cos(theta1RadActual);
            double s12 = Math.sin(theta1RadActual + theta2RadActual);
            double c12 = Math.cos(theta1RadActual + theta2RadActual);
        
            double det = L1 * L2 * Math.sin(theta2RadActual);
            double shoulderVelDegPerSec = 0.0;
            double elbowVelDegPerSec = 0.0;
        
            if (Math.abs(det) > 1e-5) {
                double invDet = 1.0 / det;
        
                // Compute angular velocities in radians
                double theta1Dot = invDet * (L2 * c12 * dydT - L2 * s12 * dxdT);
                double theta2Dot = invDet * (- (L1 * c1 + L2 * c12) * dydT + (L1 * s1 + L2 * s12) * dxdT);
        
                // Convert to deg/s
                shoulderVelDegPerSec = Math.toDegrees(theta1Dot);
                elbowVelDegPerSec = Math.toDegrees(theta2Dot);
            }
        
            // Slider velocity (m/s → rps)
            double sliderVelocityMPS = (currentSlider - lastSliderMeters) / dt;
            double sliderRPS = sliderVelocityMPS / SLIDER_METERS_PER_REV;
        
            // Output to dashboard (optional)
            SmartDashboard.putNumber("ShoulderDeg", shoulderDeg);
            SmartDashboard.putNumber("ElbowDeg", elbowDeg);
            SmartDashboard.putNumber("CurrentSlider", currentSlider);
            
            // Command joints
            arm.setJointVelocities(shoulderVelDegPerSec, elbowVelDegPerSec, sliderRPS);
        
            // Update state
            lastShoulderDeg = shoulderDeg;
            lastSliderMeters = currentSlider;
            lastElbowDeg = elbowDeg;
        
            time += dt;
        }
        
    

    @Override
    public boolean isFinished() {
        return time >= totalTime;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setJointVelocities(0.0, 0.0, 0.0);
    }
}
