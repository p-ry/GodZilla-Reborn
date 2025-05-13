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

    }@Override
    public void execute() {
        System.out.println("Time: " + time);
        if (time > totalTime) return;
    
        double t = time / totalTime;
    
        // Get target position and velocity from Bézier
        Point2D.Double pos = curve.getPositionAtTime(t);
        Point2D.Double vel = curve.getVelocityAtTime(t);
        double targetX = pos.getX();
        double targetY = pos.getY();
    
        // Offset by base
        double dx = targetX - base.getX();
        double dy = targetY - base.getY();
    
        // For debugging
        SmartDashboard.putNumber("targetX", targetX);
        SmartDashboard.putNumber("targetY", targetY);
    
        double dist = Math.hypot(dx, dy);
        SmartDashboard.putNumber("Distance", dist);
    
        if (dist > L1 + L2 + MAX_SLIDER_LENGTH || dist < Math.abs(L1 - L2)) {
            SmartDashboard.putString("Unreachable", "Unreachable");
            return;
        }
    
        double currentSlider = dist - L2;
        currentSlider = Math.max(0, Math.min(MAX_SLIDER_LENGTH, currentSlider));
        double SL2 = L2 + currentSlider;
    
        // Shoulder angle
        double baseAngle = Math.atan2(dy, dx);
        double theta1 = Math.acos(Math.max(-1.0, Math.min(1.0, (L1 * L1 + dist * dist - SL2 * SL2) / (2.0 * L1 * dist))));
        double shoulderAngle = baseAngle - theta1;
        double shoulderDeg = Math.toDegrees(shoulderAngle);
        shoulderDeg = Math.min(shoulderDeg, MAX_SHOULDER_DEG);
    
        if (!Double.isNaN(lastShoulderDeg)) {
            double delta = shoulderDeg - lastShoulderDeg;
            if (Math.abs(delta) > MAX_DELTA_SHOULDER_DEG) {
                shoulderDeg = lastShoulderDeg + Math.copySign(MAX_DELTA_SHOULDER_DEG, delta);
            }
        }
    
        // Recompute elbow position
        double theta1Rad = Math.toRadians(shoulderDeg);
        double elbowX = base.getX() + L1 * Math.cos(theta1Rad);
        double elbowY = base.getY() + L1 * Math.sin(theta1Rad);
    
        SmartDashboard.putNumber("ElbowX", elbowX);
        SmartDashboard.putNumber("ElbowY", elbowY);
    
        // Vector-based elbow angle relative to L1
        Vector2D shoulderToElbow = new Vector2D(elbowX - base.getX(), elbowY - base.getY());
        Vector2D elbowToEndEffector = new Vector2D(targetX - elbowX, targetY - elbowY);
    
        double dot = shoulderToElbow.normalize().dot(elbowToEndEffector.normalize());
        dot = Math.max(-1.0, Math.min(1.0, dot)); // Clamp
        double elbowAngleRad = Math.acos(dot);
        double cross = shoulderToElbow.cross(elbowToEndEffector);
        if (cross < 0) elbowAngleRad = -elbowAngleRad;
        double elbowDeg = Math.toDegrees(elbowAngleRad);
    
        // Velocities
        double sliderVelocityMPS = (currentSlider - lastSliderMeters) / dt;
        double sliderRPS = sliderVelocityMPS / SLIDER_METERS_PER_REV;
        double shoulderVelDegPerSec = Double.isNaN(lastShoulderDeg) ? 0.0 : (shoulderDeg - lastShoulderDeg) / dt;
        double elbowVelDegPerSec = Double.isNaN(lastElbowDeg) ? 0.0 : (elbowDeg - lastElbowDeg) / dt;
    
        // Send to arm
        SmartDashboard.putNumber("ShoulderDeg", shoulderDeg);
        SmartDashboard.putNumber("ElbowDeg", elbowDeg);
        SmartDashboard.putNumber("CurrentSlider", currentSlider);
    
        arm.setJointVelocities(shoulderVelDegPerSec, elbowVelDegPerSec, sliderRPS);
    
        // Store state
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
